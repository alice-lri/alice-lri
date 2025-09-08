#include "VerticalScanlineEstimator.h"

#include <BuildOptions.h>
#include <optional>

#include "Constants.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

// TODO split these bad boys
namespace accurate_ri {

    enum class FitConvergenceState {
        INITIAL = 0, CONVERGED = 1, CONFIRMED = 2,
    };


    std::optional<ScanlineEstimationResult> VerticalScanlineEstimator::performHeuristicFit(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
    ) {
        LOG_WARN("Heuristic fitting");
        const Eigen::ArrayXd &invRanges = points.getInvRanges()(scanlineLimits.indices);
        const Eigen::ArrayXd &phis = points.getPhis()(scanlineLimits.indices);

        const double invRangesMean = invRanges.mean();
        const double phisMean = phis.mean();

        HeuristicScanline heuristic = computeHeuristicScanline(scanlinePool, invRangesMean, phisMean);
        heuristic.offsetCi.clampBoth(-points.getMinRange(), points.getMinRange());

        const double heuristicAngle = (phis - (heuristic.offset * invRanges).asin()).mean();

        const RealMargin &angleMarginTmp = {
            .lower = (phis - (heuristic.offsetCi.lower * invRanges).asin()).mean(),
            .upper = (phis - (heuristic.offsetCi.upper * invRanges).asin()).mean()
        };

        const RealMargin &heuristicAngleCi = {
            .lower = std::min(angleMarginTmp.lower, angleMarginTmp.upper),
            .upper = std::max(angleMarginTmp.lower, angleMarginTmp.upper)
        };

        LOG_INFO(
            "Offset confidence interval: ", heuristic.offsetCi, ", Angle confidence interval: ",
            heuristicAngleCi, ", Offset: ", heuristic.offset, ", Angle: ", heuristicAngle
        );

        double offsetMargin = heuristic.offsetCi.diff() / 2;
        double angleMargin = heuristicAngleCi.diff() / 2;
        const OffsetAngleMargin houghMargin = scanlinePool.getHoughMargin();

        // Numerical stability
        offsetMargin = std::max(offsetMargin, houghMargin.offset.upper);
        angleMargin = std::max(angleMargin, houghMargin.angle.upper);

        const VerticalBounds heuristicBounds = VerticalScanlineLimits::computeErrorBounds(points, heuristic.offset);
        OffsetAngle heuristicOffsetAngle = {heuristic.offset, heuristicAngle};

        const OffsetAngleMargin heuristicMargin = {offsetMargin, offsetMargin, angleMargin, angleMargin};
        ScanlineLimits heuristicLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, heuristicBounds.final, heuristicOffsetAngle, heuristicMargin, invRangesMean
        );
        LOG_INFO("Offset: ", heuristicOffsetAngle.offset, ", Angle: ", heuristicOffsetAngle.angle);

        if (heuristicLimits.indices.size() == 0) {
            return std::nullopt;
        }

        return ScanlineEstimationResult{
            .heuristic = true,
            .uncertainty = std::numeric_limits<double>::infinity(),
            .values = heuristicOffsetAngle,
            .ci = OffsetAngleMargin{
                .offset = heuristic.offsetCi,
                .angle = heuristicAngleCi
            },
            .limits = std::move(heuristicLimits)
        };
    }

    std::optional<ScanlineEstimationResult> VerticalScanlineEstimator::estimateScanline(
        const PointArray &points, const VerticalScanlinePool &scanlinePool,
        const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        if (scanlineLimits.indices.size() == 0) {
            return std::nullopt;
        }

        if (scanlineLimits.indices.size() > 2) {
            const auto statisticFit = performStatisticalFit(points, scanlinePool, errorBounds, scanlineLimits);
            if (statisticFit) {
                return *statisticFit;
            }
        }

        if constexpr (BuildOptions::USE_VERTICAL_HEURISTICS) {
            return performHeuristicFit(points, scanlinePool, scanlineLimits);
        }

        return std::nullopt;
    }

    std::optional<ScanlineEstimationResult> VerticalScanlineEstimator::performStatisticalFit(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const VerticalBounds &errorBounds,
        const ScanlineLimits &scanlineLimits
    ) {
        ScanlineFitResult scanlineFit = tryFitScanline(points, scanlinePool, errorBounds, scanlineLimits);

        if (scanlineFit.success && !scanlineFit.ciTooWide) {
            const OffsetAngle values = {
                .offset = scanlineFit.fit->slope,
                .angle = scanlineFit.fit->intercept
            };

            OffsetAngleMargin ci = {
                .offset = {.lower = scanlineFit.fit->slopeCi[0], .upper = scanlineFit.fit->slopeCi[1]},
                .angle = {.lower = scanlineFit.fit->interceptCi[0], .upper = scanlineFit.fit->interceptCi[1]}
            };

            ci.offset.clampBoth(-points.getMinRange(), points.getMinRange());

            return ScanlineEstimationResult{
                .heuristic = false,
                .uncertainty = -scanlineFit.fit->logLikelihood,
                .values = values,
                .ci = ci,
                .limits = std::move(*scanlineFit.limits)
            };
        }

        return std::nullopt;
    }

    ScanlineFitResult VerticalScanlineEstimator::tryFitScanline(
        const PointArray &points, const VerticalScanlinePool &scanlinePool,
        const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        PROFILE_SCOPE("VerticalScanlineEstimation::tryFitScanline");
        const Eigen::ArrayXd &ranges = points.getRanges();
        VerticalBounds currentErrorBounds = errorBounds;
        ScanlineLimits currentScanlineLimits = scanlineLimits;

        std::optional<Stats::WLSResult> fitResult;
        std::optional<ScanlineLimits> limits = std::nullopt;
        FitConvergenceState state = FitConvergenceState::INITIAL;
        int8_t ciTooWideState = 0;
        bool ciTooWide = false;

        for (uint64_t attempt = 0; attempt < Constant::VERTICAL_MAX_FIT_ATTEMPTS; ++attempt) {
            const Eigen::ArrayXi &currentScanlineIndices = currentScanlineLimits.indices;

            if (currentScanlineIndices.size() <= 2) {
                break;
            }

            Eigen::ArrayX<bool> const *pointsToFitMask = &currentScanlineLimits.mask;
            Eigen::ArrayX<bool> newPointsToFitMask;

            if (state == FitConvergenceState::INITIAL) {
                newPointsToFitMask = *pointsToFitMask && (ranges >= 2);
                pointsToFitMask = &newPointsToFitMask;

                if (pointsToFitMask->count() <= 2) {
                    pointsToFitMask = &currentScanlineLimits.mask;
                }
            }

            std::vector<int32_t> pointsToFitIndicesVector;
            for (int32_t i = 0; i < pointsToFitMask->size(); ++i) {
                if ((*pointsToFitMask)[i]) {
                    pointsToFitIndicesVector.emplace_back(i);
                }
            }

            const Eigen::ArrayXi &pointsToFitIndices = Eigen::ArrayXi::Map(
                pointsToFitIndicesVector.data(), static_cast<Eigen::Index>(pointsToFitIndicesVector.size())
            );
            const Eigen::ArrayXd &invRanges = points.getInvRanges();
            const Eigen::ArrayXd &phis = points.getPhis();

            const Eigen::ArrayXd &invRangesFiltered = invRanges(pointsToFitIndices);
            const Eigen::ArrayXd &phisFiltered = phis(pointsToFitIndices);
            const Eigen::ArrayXd &boundsFiltered = currentErrorBounds.final(pointsToFitIndices);

            fitResult = Stats::wlsBoundsFit(invRangesFiltered, phisFiltered, boundsFiltered);
            double offsetCiWidth = fitResult->slopeCi(1) - fitResult->slopeCi(0);
            int32_t pointFitCount = static_cast<int32_t>(pointsToFitIndices.size());

            LOG_INFO(
                "Model fit iteration; Offset: ", fitResult->slope, ", Angle: ", fitResult->intercept,
                " using ", pointFitCount, " points, Convergence state: ", static_cast<int>(state), ","
            );

            if (offsetCiWidth > 1e-2) {
                ciTooWideState++;

                if (ciTooWideState >= 2) {
                    LOG_WARN("CI too wide: ", offsetCiWidth);
                    ciTooWide = true;
                    break;
                }
            } else {
                ciTooWideState = 0;
            }

            currentErrorBounds = VerticalScanlineLimits::computeErrorBounds(points, fitResult->slope);

            const OffsetAngleMargin margin = scanlinePool.getHoughMargin();
            const double meanInvRanges = invRangesFiltered.mean();

            const OffsetAngle scanlineAttributes = {.offset = fitResult->slope, .angle = fitResult->intercept};
            const ScanlineLimits newLimits = VerticalScanlineLimits::computeScanlineLimits(
                points, currentErrorBounds.final, scanlineAttributes, margin, meanInvRanges
            );

            LOG_INFO(
                "Minimum limit width (fit): ", (newLimits.upperLimit - newLimits.lowerLimit).minCoeff()
            );

            if ((newLimits.mask == currentScanlineLimits.mask).all()) {
                if (state == FitConvergenceState::CONVERGED) {
                    state = FitConvergenceState::CONFIRMED;
                    break;
                }

                state = FitConvergenceState::CONVERGED;
            }

            currentScanlineLimits = newLimits;
        }

        return {
            .fit = state == FitConvergenceState::CONFIRMED ? fitResult : std::nullopt,
            .limits = state == FitConvergenceState::CONFIRMED
                          ? std::make_optional(currentScanlineLimits)
                          : std::nullopt,
            .success = state == FitConvergenceState::CONFIRMED,
            .ciTooWide = ciTooWide,
        };
    }

    HeuristicScanline VerticalScanlineEstimator::computeHeuristicScanline(
        const VerticalScanlinePool &scanlinePool, const double invRangesMean, const double phisMean
    ) {
        std::optional<uint32_t> closestScanlineIdTop = std::nullopt;
        std::optional<uint32_t> closestScanlineIdBottom = std::nullopt;
        double closestScanlineTopDistance = std::numeric_limits<double>::infinity();
        double closestScanlineBottomDistance = std::numeric_limits<double>::infinity();

        scanlinePool.forEachScanline(
            [&](const ScanlineInfo &scanline) {
                uint32_t scanlineId = scanline.id;

                const double scanlinePhi = std::asin(scanline.values.offset * invRangesMean) + scanline.values.angle;
                if (scanlinePhi > phisMean) {
                    const double distance = scanlinePhi - phisMean;
                    if (distance < closestScanlineTopDistance) {
                        closestScanlineIdTop = scanlineId;
                        closestScanlineTopDistance = distance;
                    }
                }
                if (scanlinePhi < phisMean) {
                    const double distance = phisMean - scanlinePhi;
                    if (distance < closestScanlineBottomDistance) {
                        closestScanlineIdBottom = scanlineId;
                        closestScanlineBottomDistance = distance;
                    }
                }
            }
        );

        std::vector<uint32_t> validScanlineIds;
        if (closestScanlineIdTop) {
            validScanlineIds.emplace_back(*closestScanlineIdTop);
        }
        if (closestScanlineIdBottom) {
            validScanlineIds.emplace_back(*closestScanlineIdBottom);
        }

        assert(!validScanlineIds.empty() && "No valid scanlines found");

        std::vector<double> offsetDiffs;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;
            offsetDiffs.emplace_back(offsetDiff);
        }

        const double maxOffsetDiff = std::ranges::max(offsetDiffs);
        double meanOffset = 0;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            meanOffset += scanline.values.offset / static_cast<double>(validScanlineIds.size());
        }

        return HeuristicScanline{
            .offset = meanOffset,
            .offsetCi = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2}
        };
    }
} // accurate_ri
