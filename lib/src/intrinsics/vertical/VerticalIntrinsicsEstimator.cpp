#include "VerticalIntrinsicsEstimator.h"

#include <accurate_ri/accurate_ri.hpp>
#include <algorithm>
#include <queue>
#include <ranges>
#include "helper/VerticalLogging.h"
#include "utils/Utils.h"
#include "VerticalStructs.h"
#include "utils/TestUtils.h"
#include "utils/Timer.h"
#include <nlohmann/json.hpp>

#include "Constants.h"
#include "BuildOptions.h"
#include "math/Stats.h"

namespace accurate_ri {

    enum class FitConvergenceState {
        INITIAL = 0, CONVERGED = 1, CONFIRMED = 2,
    };

    // TODO probably make dedicated structs for all the tuples
    VerticalIntrinsicsResult VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        initScanlinePool(points);
        VerticalLogging::printHeaderDebugInfo(points, *scanlinePool);

        scanlinePool->performPrecomputations(points);

        int64_t iteration = -1;
        uint32_t currentScanlineId = 0;
        EndReason endReason = EndReason::ALL_ASSIGNED;

        while (scanlinePool->anyUnassigned()) {
            iteration++;
            if (iteration > Constant::VERTICAL_MAX_ITERATIONS) {
                endReason = EndReason::MAX_ITERATIONS;
                LOG_WARN("Warning: Maximum iterations reached");
                break;
            }

            const auto &houghEstimationOpt = scanlinePool->performHoughEstimation();

            if (!houghEstimationOpt) {
                endReason = EndReason::NO_MORE_PEAKS;
                LOG_WARN("Warning: No more peaks found");
                break;
            }

            const HoughCell &houghMax = houghEstimationOpt->cell;
            const OffsetAngle &houghValues = houghMax.maxValues;
            const OffsetAngleMargin &margin = houghEstimationOpt->margin;

            LOG_INFO("ITERATION ", iteration);
            LOG_INFO(
                "Offset: ", houghValues.offset, ", Angle: ", houghValues.angle, ", Votes: ", houghMax.votes,
                ", Hash: ", houghMax.hash, ", Hough indices: [", houghMax.maxAngleIndex, "   ", houghMax.maxOffsetIndex,
                "]"
            );

            VerticalBounds errorBounds = computeErrorBounds(points, houghValues.offset);
            ScanlineLimits scanlineLimits = computeScanlineLimits(
                points, errorBounds.final, houghValues, margin, 0
            );

            LOG_INFO(
                "Minimum limit width (Hough): ", (scanlineLimits.upperLimit - scanlineLimits.lowerLimit).minCoeff()
            );

            VerticalLogging::plotDebugInfo(
                points, scanlineLimits, scanlinePool->getPointsScanlinesIds(), iteration, "hough_", houghValues, 0
            );

            // TODO houghvalues and scanline limits should not be used beyond this point, refactor to avoid


            const auto &estimationResultOpt = estimateScanline(points, errorBounds, scanlineLimits);

            if (!estimationResultOpt) {
                LOG_INFO("Fit failed: True, Points in scanline: ", scanlineLimits.indices.size());
                LOG_INFO("");

                scanlinePool->invalidateByHash(houghMax.hash);
                continue;
            }

            ScanlineEstimationResult scanlineEstimation = *estimationResultOpt;
            const OffsetAngle &maxValues = scanlineEstimation.values;

            VerticalLogging::plotDebugInfo(
                points, scanlineEstimation.limits, scanlinePool->getPointsScanlinesIds(), iteration, "fit_", maxValues,
                scanlineEstimation.uncertainty
            );

            ScanlineAngleBounds angleBounds = {
                .bottom = {
                    .lower = scanlineEstimation.ci.angle.lower + asin(
                                 scanlineEstimation.ci.offset.lower / points.getMaxRange()
                             ),
                    .upper = scanlineEstimation.ci.angle.lower + asin(
                                 scanlineEstimation.ci.offset.lower / points.getMinRange()
                             )
                },
                .top = {
                    .lower = scanlineEstimation.ci.angle.upper + asin(
                                 scanlineEstimation.ci.offset.upper / points.getMaxRange()
                             ),
                    .upper = scanlineEstimation.ci.angle.upper + asin(
                                 scanlineEstimation.ci.offset.upper / points.getMinRange()
                             )
                }
            };

            if constexpr (BuildOptions::USE_SCANLINE_CONFLICT_SOLVER) {
                bool keepScanline = conflictSolver.performScanlineConflictResolution(
                    *scanlinePool, angleBounds, scanlineEstimation, currentScanlineId, houghMax
                );

                if (!keepScanline) {
                    continue;
                }
            }

            if (scanlineEstimation.uncertainty < Constant::FULL_CERTAINTY_THRESHOLD) {
                scanlinePool->invalidateByPoints(points, scanlineEstimation.limits.indices);
            } else {
                scanlinePool->invalidateByHash(houghMax.hash);
            }

            ScanlineInfo scanlineInfo = ScanlineInfo{
                .id = currentScanlineId,
                .pointsCount = static_cast<uint64_t>(scanlineEstimation.limits.indices.size()),
                .values = maxValues,
                .ci = scanlineEstimation.ci,
                .theoreticalAngleBounds = angleBounds,
                .dependencies = std::move(scanlineEstimation.dependencies),
                .uncertainty = scanlineEstimation.uncertainty,
                .houghVotes = houghMax.votes,
                .houghHash = houghMax.hash
            };

            scanlinePool->assignScanline(std::move(scanlineInfo), scanlineEstimation.limits.indices);

            LOG_INFO(
                "Scanline ", currentScanlineId, " assigned with ", scanlineEstimation.limits.indices.size(), " points"
            );
            LOG_INFO(
                "Scanline parameters: Offset: ", maxValues.offset, ", Angle: ", maxValues.angle, ", Votes: ",
                houghMax.votes,
                ", Count: ", scanlineEstimation.limits.indices.size(),
                ", Lower min theoretical angle: ", angleBounds.bottom.lower, ", Lower max theoretical angle: ",
                angleBounds.bottom.upper,
                ", Upper min theoretical angle: ", angleBounds.top.lower, ", Upper max theoretical angle: ",
                angleBounds.top.upper,
                ", Uncertainty: ", scanlineEstimation.uncertainty
            );
            LOG_INFO("Number of unassigned points: ", scanlinePool->getUnassignedPoints());
            LOG_INFO("");

            currentScanlineId++;
        }

        int64_t unassignedPoints = scanlinePool->getUnassignedPoints();

        // TODO last resort assignment is disabled, maybe implement?
        if (scanlinePool->anyUnassigned()) {
            LOG_WARN("Warning: Found ", unassignedPoints, " spurious points");
        }

        FullScanlines fullScanlines = scanlinePool->extractFullSortedScanlines();

        LOG_INFO("Number of scanlines: ", fullScanlines.scanlines.size());
        LOG_INFO("Number of unassigned points: ", unassignedPoints);

        VerticalIntrinsicsResult result = {
            .iterations = static_cast<uint32_t>(iteration),
            .scanlinesCount = static_cast<uint32_t>(fullScanlines.scanlines.size()),
            .unassignedPoints = static_cast<uint32_t>(unassignedPoints),
            .pointsCount = static_cast<uint32_t>(points.size()),
            .endReason = endReason,
            .fullScanlines = std::move(fullScanlines)
        };

        return result;
    }

    void VerticalIntrinsicsEstimator::initScanlinePool(const PointArray &points) {
        double offsetMax = std::min(std::ranges::min(points.getRanges()), Constant::MAX_OFFSET) - Constant::OFFSET_STEP;
        double offsetMin = -offsetMax;

        double angleMax = M_PI / 2 - Constant::ANGLE_STEP;
        double angleMin = -angleMax;

        scanlinePool = std::make_unique<VerticalScanlinePool>(
            offsetMin, offsetMax, Constant::OFFSET_STEP, angleMin, angleMax, Constant::ANGLE_STEP
        );
    }

    // TODO precompute on PointArray
    VerticalBounds VerticalIntrinsicsEstimator::computeErrorBounds(
        const PointArray &points, const double offset
    ) {
        PROFILE_SCOPE("VerticalIntrinsicsEstimator::computeErrorBounds");
        double coordsEps = points.getCoordsEps();
        const auto &zs = points.getZ();
        const auto &rangesXy = points.getRangesXy();
        const auto &ranges = points.getRanges();

        const auto &rangeXySquared = rangesXy.square();
        const auto &rangeSquared = ranges.square();

        const double rangesBound = coordsEps * std::sqrt(3);
        const double rangesXyBound = coordsEps * std::sqrt(2);

        VerticalBounds result;

        const auto phisBoundNumerator = rangesXyBound * zs.cwiseAbs() + coordsEps * rangesXy;
        const auto phisBoundDenominator = rangeXySquared - rangesXyBound * rangesXy;
        const auto correctionBoundNumerator = std::abs(offset) * rangesBound;
        const auto correctionBoundDenominator = rangeSquared - rangesBound * ranges;

        result.phis = phisBoundNumerator / phisBoundDenominator;
        result.correction = correctionBoundNumerator / correctionBoundDenominator;
        result.final = result.phis + result.correction;

        return result;
    }

    // TODO this could be a generic line thing. Take a look at conceptually section in scanlimie limits notebook
    ScanlineLimits VerticalIntrinsicsEstimator::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const OffsetAngleMargin &margin, const double invRangesShift
    ) const {
        PROFILE_SCOPE("VerticalIntrinsicsEstimator::computeScanlineLimits");
        const auto &inv = points.getInvRanges();
        const auto &phi = points.getPhis();
        const double offset = scanlineAttributes.offset;
        const double angle = scanlineAttributes.angle;

        const auto sinUpper = ((offset + margin.offset.upper) * inv.array()).min(1).max(-1);
        const auto sinLower = ((offset - margin.offset.lower) * inv.array()).min(1).max(-1);

        Eigen::ArrayXd upper = angle + sinUpper + margin.angle.upper + errorBounds.array();
        Eigen::ArrayXd lower = angle + sinLower - margin.angle.lower - errorBounds.array();
        Eigen::ArrayX<bool> mask = (lower <= phi) && (phi <= upper);
        Eigen::ArrayXi idx(mask.count());

        int k = 0;
        for (Eigen::Index i = 0; i < mask.size(); ++i) {
            if (mask[i]) {
                idx[k++] = static_cast<int>(i);
            }
        }

        return { std::move(idx), std::move(mask), std::move(lower), std::move(upper) };
    }

    // TODO split this function
    std::optional<ScanlineEstimationResult> VerticalIntrinsicsEstimator::estimateScanline(
        const PointArray &points, const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        bool requiresHeuristicFitting = true;

        if (scanlineLimits.indices.size() == 0) {
            return std::nullopt;
        }

        if (scanlineLimits.indices.size() > 2) {
            ScanlineFitResult scanlineFit = tryFitScanline(points, errorBounds, scanlineLimits);
            requiresHeuristicFitting = scanlineFit.ciTooWide;

            if (scanlineFit.success && !requiresHeuristicFitting) {
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
                    .uncertainty = scanlineFit.fit->aic,
                    .values = values,
                    .ci = ci,
                    .limits = std::move(*scanlineFit.limits),
                    .dependencies = std::vector<uint32_t>()
                };
            }
        }

        if constexpr (!BuildOptions::USE_VERTICAL_HEURISTICS) {
            return std::nullopt;
        }

        if (requiresHeuristicFitting) {
            LOG_WARN("Heuristic fitting");
            const Eigen::ArrayXd &invRanges = points.getInvRanges()(scanlineLimits.indices);
            const Eigen::ArrayXd &phis = points.getPhis()(scanlineLimits.indices);

            const double invRangesMean = invRanges.mean();
            const double phisMean = phis.mean();

            HeuristicScanline heuristic = computeHeuristicScanline(invRangesMean, phisMean);
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

            // Numerical stability
            offsetMargin = std::max(offsetMargin, 1e-6);
            angleMargin = std::max(angleMargin, 1e-6);

            const VerticalBounds heuristicBounds = computeErrorBounds(points, heuristic.offset);
            OffsetAngle heuristicOffsetAngle = {heuristic.offset, heuristicAngle};

            const OffsetAngleMargin heuristicMargin = {offsetMargin, offsetMargin, angleMargin, angleMargin};
            ScanlineLimits heuristicLimits = computeScanlineLimits(
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
                .ci = OffsetAngleMargin {
                    .offset = heuristic.offsetCi,
                    .angle = heuristicAngleCi
                },
                .limits = std::move(heuristicLimits),
                .dependencies = std::vector<uint32_t>()
            };
        }

        return std::nullopt;
    }

    ScanlineFitResult VerticalIntrinsicsEstimator::tryFitScanline(
        const PointArray &points, const VerticalBounds &errorBounds,
        const ScanlineLimits &scanlineLimits
    ) const {
        PROFILE_SCOPE("VerticalIntrinsicsEstimator::tryFitScanline");
        const Eigen::ArrayXd &ranges = points.getRanges();
        VerticalBounds currentErrorBounds = errorBounds;
        ScanlineLimits currentScanlineLimits = scanlineLimits;

        std::optional<Stats::WLSResult> fitResult = std::nullopt;
        std::optional<ScanlineLimits> limits = std::nullopt;
        FitConvergenceState state = FitConvergenceState::INITIAL;
        bool ciTooWide = false;

        for (uint64_t attempt = 0; attempt < Constant::VERTICAL_MAX_FIT_ATTEMPTS; ++attempt) {
            const Eigen::ArrayXi &currentScanlineIndices = currentScanlineLimits.indices;

            if (currentScanlineIndices.size() <= 2) {
                break;
            }

            // TODO maybe avoid using pointers here through move semantics
            Eigen::ArrayX<bool> const *pointsToFitMask = &currentScanlineLimits.mask;
            Eigen::ArrayX<bool> newPointsToFitMask;

            if (state == FitConvergenceState::INITIAL) {
                newPointsToFitMask = *pointsToFitMask && (ranges >= 2); // TODO tricky
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
                pointsToFitIndicesVector.data(), pointsToFitIndicesVector.size()
            );
            const Eigen::ArrayXd &invRanges = points.getInvRanges();
            const Eigen::ArrayXd &phis = points.getPhis();

            // TODO check if we can avoid these copies by lazy evaluating inside the performFit function
            const Eigen::ArrayXd &invRangesFiltered = invRanges(pointsToFitIndices);
            const Eigen::ArrayXd &phisFiltered = phis(pointsToFitIndices);
            const Eigen::ArrayXd &boundsFiltered = currentErrorBounds.final(pointsToFitIndices);

            fitResult = Stats::wlsBoundsFit(invRangesFiltered, phisFiltered, boundsFiltered);
            double offsetCiWidth = fitResult->slopeCi(1) - fitResult->slopeCi(0);
            int32_t pointFitCount = pointsToFitIndices.size();

            LOG_INFO(
                "Model fit iteration; Offset: ", fitResult->slope, ", Angle: ", fitResult->intercept,
                " using ", pointFitCount, " points, Convergence state: ", static_cast<int>(state), ","
            );

            // TODO review and justify these constants
            // TODO maybe this is not even necessary, maybe we can just rely on heuristics and conflict resolution
            if (offsetCiWidth > 1e-2) {
                LOG_WARN("CI too wide: ", offsetCiWidth);
                ciTooWide = true;
                break;
            }

            // TODO we could save memory reallocation here (perhaps by passing buffer by ref to computeErrorBounds)
            currentErrorBounds = computeErrorBounds(points, fitResult->slope);

            const OffsetAngleMargin margin = scanlinePool->getHoughMargin();
            const double meanInvRanges = invRangesFiltered.mean();

            const OffsetAngle scanlineAttributes = {.offset = fitResult->slope, .angle = fitResult->intercept};
            const ScanlineLimits newLimits = computeScanlineLimits(
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

    HeuristicScanline VerticalIntrinsicsEstimator::computeHeuristicScanline(
        const double invRangesMean, const double phisMean
    ) const {
        std::optional<uint32_t> closestScanlineIdTop = std::nullopt;
        std::optional<uint32_t> closestScanlineIdBottom = std::nullopt;
        double closestScanlineTopDistance = std::numeric_limits<double>::infinity();
        double closestScanlineBottomDistance = std::numeric_limits<double>::infinity();

        scanlinePool->forEachScanline(
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
            const auto &scanline = scanlinePool->getScanlineById(scanlineId);
            double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;
            offsetDiffs.emplace_back(offsetDiff);
        }

        const double maxOffsetDiff = std::ranges::max(offsetDiffs);
        double meanOffset = 0;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlinePool->getScanlineById(scanlineId);
            meanOffset += scanline.values.offset / static_cast<double>(validScanlineIds.size());
        }

        return HeuristicScanline{
            .offset = meanOffset,
            .offsetCi = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2},
            .dependencies = validScanlineIds
        };
    }
} // namespace accurate_ri
