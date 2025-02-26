#include "VerticalIntrinsicsEstimator.h"
#include <algorithm>
#include <queue>
#include <ranges>
#include <unordered_set>
#include <boost/math/distributions/students_t.hpp>
#include "helper/VerticalLogging.h"
#include "utils/Utils.h"
#include "VerticalStructs.h"
#include "utils/TestUtils.h"
#include "utils/Timer.h"
#include <nlohmann/json.hpp>

#include "intrinsics/vertical/helper/JsonConverters.h"
#include "math/Stats.h"

namespace accurate_ri {
    // TODO extract these constants somewhere
    constexpr uint64_t MAX_ITERATIONS = 10000;
    constexpr double MAX_OFFSET = 0.5;
    constexpr double OFFSET_STEP = 1e-3;
    constexpr double ANGLE_STEP = 1e-4;
    constexpr uint64_t MAX_FIT_ATTEMPTS = 10;

    enum class FitConvergenceState {
        INITIAL = 0, CONVERGED = 1, CONFIRMED = 2,
    };

    // TODO probably make dedicated structs for all the tuples
    VerticalIntrinsicsResult VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        initHough(points);
        hough->computeAccumulator(points);

        LOG_DEBUG("Dimensions of testHash: ", hough->getYCount(), " x ", hough->getXCount());

        VerticalLogging::printHeaderDebugInfo(points, *hough);

        pointsScanlinesIds = Eigen::ArrayXi::Ones(points.size()) * -1;
        unassignedPoints = points.size();
        int64_t iteration = -1;
        uint32_t currentScanlineId = 0;
        EndReason endReason = EndReason::ALL_ASSIGNED;

        while (unassignedPoints > 0) {
            iteration++;
            if (iteration > MAX_ITERATIONS) {
                endReason = EndReason::MAX_ITERATIONS;
                LOG_WARN("Warning: Maximum iterations reached");
                break;
            }

            double averageOffset = 0;
            for (const ScanlineInfo &info: scanlineInfoMap | std::views::values) {
                averageOffset += info.values.offset / static_cast<double>(scanlineInfoMap.size());
            }

            const std::optional<HoughCell> houghMaxOpt = hough->findMaximum(averageOffset);

            if (!houghMaxOpt) {
                endReason = EndReason::NO_MORE_PEAKS;
                LOG_WARN("Warning: No more peaks found");
                break;
            }

            const HoughCell houghMax = *houghMaxOpt;
            OffsetAngle maxValues = houghMax.maxValues;

            LOG_INFO("ITERATION ", iteration);
            LOG_INFO(
                "Offset: ", maxValues.offset, ", Angle: ", maxValues.angle, ", Votes: ", houghMax.votes,
                ", Hash: ", houghMax.hash, ", Hough indices: [", houghMax.maxAngleIndex, "   ", houghMax.maxOffsetIndex,
                "]"
            );

            const OffsetAngleMargin margin = {
                {hough->getXStep(), hough->getXStep()},
                {hough->getYStep(), hough->getYStep()}
            };

            VerticalBounds errorBounds = computeErrorBounds(points, maxValues.offset);
            ScanlineLimits scanlineLimits = computeScanlineLimits(
                points, errorBounds.final, maxValues, margin, 0
            );

            LOG_INFO(
                "Minimum limit width (Hough): ", (scanlineLimits.upperLimit - scanlineLimits.lowerLimit).minCoeff()
            );

            VerticalLogging::plotDebugInfo(
                points, scanlineLimits, pointsScanlinesIds, iteration, "hough_", maxValues, 0
            );

            const auto &estimationResultOpt = estimateScanline(points, errorBounds, scanlineLimits);

            if (!estimationResultOpt) {
                LOG_INFO(
                    "Fit failed: True, Points in scanline: ", scanlineLimits.indices.size()
                );
                LOG_INFO("");

                hough->eraseByHash(houghMax.hash);
                continue;
            }

            const ScanlineEstimationResult &scanlineEstimation = *estimationResultOpt;

            VerticalLogging::plotDebugInfo(
                points, scanlineEstimation.limits, pointsScanlinesIds, iteration, "fit_", maxValues,
                scanlineEstimation.uncertainty
            );

            const ScanlineAngleBounds &angleBounds = {
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


            bool keepScanline = performScanlineConflictResolution(
                angleBounds, scanlineEstimation, currentScanlineId, houghMax
            );

            if (!keepScanline) {
                continue;
            }

            // TODO this is a bit hacky wacky
            if (scanlineEstimation.uncertainty < -500) {
                hough->eraseWhere(points, scanlineEstimation.limits.indices);
            } else {
                hough->eraseByHash(houghMax.hash);
            }

            pointsScanlinesIds(scanlineEstimation.limits.indices) = currentScanlineId;
            unassignedPoints -= scanlineEstimation.limits.indices.size();

            for (const auto &dependency: scanlineEstimation.dependencies) {
                reverseScanlinesDependencyMap.emplace(dependency, currentScanlineId);
            }

            scanlineInfoMap.emplace(
                currentScanlineId, ScanlineInfo{
                    .scanlineId = currentScanlineId,
                    .pointsCount = static_cast<uint64_t>(scanlineEstimation.limits.indices.size()),
                    .values = maxValues,
                    .ci = scanlineEstimation.ci,
                    .theoreticalAngleBounds = std::move(angleBounds),
                    .dependencies = std::move(scanlineEstimation.dependencies),
                    .uncertainty = scanlineEstimation.uncertainty,
                    .houghVotes = houghMax.votes,
                    .houghHash = houghMax.hash
                }
            );

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
            LOG_INFO("Number of unassigned points: ", unassignedPoints);
            LOG_INFO("");

            currentScanlineId++;
        }

        // TODO last resort assignment is disabled, maybe implement?
        if (unassignedPoints > 0) {
            LOG_WARN("Warning: Found ", unassignedPoints, " spurious points");
        }

        std::vector<ScanlineInfo> sortedScanlines;
        for (ScanlineInfo &scanlineInfo: scanlineInfoMap | std::views::values) {
            sortedScanlines.emplace_back(std::move(scanlineInfo));
        }
        scanlineInfoMap.clear();

        std::ranges::sort(
            sortedScanlines, [](const ScanlineInfo &a, const ScanlineInfo &b) {
                return a.values.angle < b.values.angle;
            }
        );


        std::unordered_map<uint32_t, uint32_t> oldIdsToNewIdsMap;
        for (uint32_t i = 0; i < sortedScanlines.size(); ++i) {
            oldIdsToNewIdsMap.emplace(sortedScanlines[i].scanlineId, i);
        }

        for (auto &scanlineInfo: sortedScanlines) {
            scanlineInfo.scanlineId = oldIdsToNewIdsMap[scanlineInfo.scanlineId];
            std::ranges::transform(
                scanlineInfo.dependencies, scanlineInfo.dependencies.begin(), [&oldIdsToNewIdsMap](const uint32_t id) {
                    return oldIdsToNewIdsMap[id];
                }
            );
        }

        std::ranges::transform(
            pointsScanlinesIds, pointsScanlinesIds.begin(), [&oldIdsToNewIdsMap](const int32_t id) {
                return (id >= 0) ? oldIdsToNewIdsMap[id] : -1;
            }
        );

        LOG_INFO("Number of scanlines: ", sortedScanlines.size());
        LOG_INFO("Number of unassigned points: ", unassignedPoints);

        VerticalIntrinsicsResult result = {
            .iterations = static_cast<uint32_t>(iteration),
            .scanlinesCount = static_cast<uint32_t>(sortedScanlines.size()),
            .unassignedPoints = static_cast<uint32_t>(unassignedPoints),
            .pointsCount = static_cast<uint32_t>(points.size()),
            .endReason = endReason,
            .scanlines = std::move(sortedScanlines),
            .pointsScanlinesIds = std::move(pointsScanlinesIds)
        };

        writeToJson(result);

        return result;
    }

    void VerticalIntrinsicsEstimator::initHough(const PointArray &points) {
        double offsetMax = std::min(std::ranges::min(points.getRanges()), MAX_OFFSET) - OFFSET_STEP;
        double offsetMin = -offsetMax;

        double angleMax = M_PI / 2 - ANGLE_STEP;
        double angleMin = -angleMax;

        hough = std::make_unique<HoughTransform>(offsetMin, offsetMax, OFFSET_STEP, angleMin, angleMax, ANGLE_STEP);
    }

    // TODO precompute on PointArray
    // TODO review equation and make code cleaner
    VerticalBounds VerticalIntrinsicsEstimator::computeErrorBounds(
        const PointArray &points, const double offset
    ) {
        double coordsEps = points.getCoordsEps();
        const auto &zs = points.getZ();
        const auto &rangesXy = points.getRangesXy();
        const auto &ranges = points.getRanges();

        const Eigen::ArrayXd rangeXySquared = rangesXy.array().square();
        const Eigen::ArrayXd rangeSquared = ranges.array().square();
        const Eigen::ArrayXd zsOverRangesXy = zs.array() / rangesXy.array();
        const Eigen::ArrayXd sqrtFactor = 1 + zsOverRangesXy.square();

        const Eigen::ArrayXd phisUpperBound = (coordsEps * std::sqrt(2) * zs.cwiseAbs() + coordsEps * rangesXy).array()
                                              / (rangeXySquared.array() * sqrtFactor.array());

        const Eigen::ArrayXd correctionUpperBound = offset * coordsEps * std::sqrt(3)
                                                    / (rangeSquared.array() * (1 - (offset / ranges.array()).square()).
                                                       sqrt());

        const Eigen::ArrayXd finalUpperBound = phisUpperBound + correctionUpperBound;


        return {phisUpperBound, correctionUpperBound, finalUpperBound};
    }

    // TODO split this function
    // TODO margins need to be arguments
    ScanlineLimits VerticalIntrinsicsEstimator::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const OffsetAngleMargin &margin, const double invRangesShift
    ) const {
        const auto &invRanges = points.getInvRanges();
        const auto &phis = points.getPhis();
        const auto offset = scanlineAttributes.offset;
        const auto angle = scanlineAttributes.angle;

        const Eigen::ArrayXd upperArcsinArg = ((offset + margin.offset.upper) * invRanges.array()).min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArg = ((offset - margin.offset.lower) * invRanges.array()).min(1).max(-1);

        const Eigen::ArrayXd upperArcsinArgShifted =
                ((offset + margin.offset.upper) * (invRanges.array() - invRangesShift)).min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArgShifted =
                ((offset - margin.offset.lower) * (invRanges.array() - invRangesShift)).min(1).max(-1);

        const Eigen::ArrayXd upperArcsin = upperArcsinArg.array().asin();
        const Eigen::ArrayXd lowerArcsin = lowerArcsinArg.array().asin();

        const Eigen::ArrayXd upperArcsinShifted = upperArcsinArgShifted.array().asin();
        const Eigen::ArrayXd lowerArcsinShifted = lowerArcsinArgShifted.array().asin();

        const Eigen::ArrayXd deltaUpper = upperArcsin - upperArcsinShifted;
        const Eigen::ArrayXd deltaLower = lowerArcsin - lowerArcsinShifted;

        const Eigen::ArrayXd scanlineUpperLimitTmp = upperArcsinShifted + angle;
        const Eigen::ArrayXd scanlineLowerLimitTmp = lowerArcsinShifted + angle;

        Eigen::ArrayXd scanlineUpperLimit = scanlineUpperLimitTmp.array().max(scanlineLowerLimitTmp.array());
        Eigen::ArrayXd scanlineLowerLimit = scanlineLowerLimitTmp.array().min(scanlineUpperLimitTmp.array());

        scanlineUpperLimit += deltaUpper + margin.angle.upper + errorBounds.array();
        scanlineLowerLimit += deltaLower - margin.angle.lower - errorBounds.array();

        auto scanlineIndices = Eigen::ArrayXi(points.size());
        Eigen::ArrayX<bool> mask = scanlineLowerLimit.array() <= phis.array()
                                   && phis.array() <= scanlineUpperLimit.array();

        int32_t count = 0;
        for (int32_t i = 0; i < points.size(); ++i) {
            if (mask[i]) {
                scanlineIndices[count++] = i;
            }
        }

        scanlineIndices.conservativeResize(count);

        return {scanlineIndices, mask, scanlineLowerLimit, scanlineUpperLimit};
    }

    std::optional<ScanlineEstimationResult> VerticalIntrinsicsEstimator::estimateScanline(
        const PointArray &points, const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        bool requiresHeuristicFitting = false;

        if (scanlineLimits.indices.size() == 0) {
            return std::nullopt;
        }

        if (scanlineLimits.indices.size() > 2) {
            const ScanlineFitResult &scanlineFit = tryFitScanline(points, errorBounds, scanlineLimits);
            requiresHeuristicFitting = scanlineFit.ciTooWide;

            if (scanlineFit.success and !requiresHeuristicFitting) {
                const OffsetAngle values = {
                    .offset = scanlineFit.fit->slope,
                    .angle = scanlineFit.fit->intercept
                };

                const OffsetAngleMargin ci = {
                    .offset = {.lower = scanlineFit.fit->slopeCi[0], .upper = scanlineFit.fit->slopeCi[1]},
                    .angle = {.lower = scanlineFit.fit->interceptCi[0], .upper = scanlineFit.fit->interceptCi[1]}
                };

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

        if (requiresHeuristicFitting) {
            LOG_WARN("Heuristic fitting");
            const Eigen::ArrayXd &invRanges = points.getInvRanges()(scanlineLimits.indices);
            const Eigen::ArrayXd &phis = points.getPhis()(scanlineLimits.indices);

            const double invRangesMean = invRanges.mean();
            const double phisMean = phis.mean();

            const HeuristicScanline &heuristic = computeHeuristicScanline(invRangesMean, phisMean);
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

            const VerticalBounds &heuristicBounds = computeErrorBounds(points, heuristic.offset);
            const OffsetAngle &heuristicOffsetAngle = {heuristic.offset, heuristicAngle};

            const OffsetAngleMargin &heuristicMargin = {offsetMargin, offsetMargin, angleMargin, angleMargin};
            const ScanlineLimits &heuristicLimits = computeScanlineLimits(
                points, heuristicBounds.final, heuristicOffsetAngle, heuristicMargin, invRangesMean
            );
            LOG_INFO("Offset: ", heuristicOffsetAngle.offset, ", Angle: ", heuristicOffsetAngle.angle);

            return ScanlineEstimationResult{
                .heuristic = true,
                .uncertainty = std::numeric_limits<double>::infinity(),
                .values = std::move(heuristicOffsetAngle),
                .ci = std::move(heuristicMargin),
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
        const Eigen::ArrayXd &ranges = points.getRanges().array();
        VerticalBounds currentErrorBounds = errorBounds;
        ScanlineLimits currentScanlineLimits = scanlineLimits;

        std::optional<Stats::WLSResult> fitResult = std::nullopt;
        std::optional<ScanlineLimits> limits = std::nullopt;
        FitConvergenceState state = FitConvergenceState::INITIAL;
        bool ciTooWide = false;

        for (uint64_t attempt = 0; attempt < MAX_FIT_ATTEMPTS; ++attempt) {
            const Eigen::ArrayXi &currentScanlineIndices = currentScanlineLimits.indices;

            if (currentScanlineIndices.size() <= 2) {
                break;
            }

            // TODO maybe avoid using pointers here through move semantics
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
                pointsToFitIndicesVector.data(), pointsToFitIndicesVector.size()
            );
            const Eigen::ArrayXd &invRanges = points.getInvRanges();
            const Eigen::ArrayXd &phis = points.getPhis();

            // TODO, careful, maybe this does not work. But: https://eigen.tuxfamily.org/dox/group__TutorialSlicingIndexing.html#title5
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
            if (offsetCiWidth > std::max(0.05 * fitResult->slope, 1e-2)) {
                LOG_WARN("CI too wide: ", offsetCiWidth);
                ciTooWide = true;
                break;
            }

            // TODO we could save memory reallocation here (perhaps by passing buffer by ref to computeErrorBounds)
            currentErrorBounds = computeErrorBounds(points, fitResult->slope);

            // TODO take into account that strict fit was removed here
            double upperOffsetMargin = fitResult->slopeCi(1) - fitResult->slope;
            double lowerOffsetMargin = fitResult->slope - fitResult->slopeCi(0);
            double upperAngleMargin = fitResult->interceptCi(1) - fitResult->intercept;
            double lowerAngleMargin = fitResult->intercept - fitResult->interceptCi(0);

            // TODO 1e-6 can be justified for numerical stability, but well need to review 5e-4
            upperOffsetMargin = std::max(upperOffsetMargin, 5e-4);
            lowerOffsetMargin = std::max(lowerOffsetMargin, 5e-4);
            upperAngleMargin = std::max(upperAngleMargin, 1e-6);
            lowerAngleMargin = std::max(lowerAngleMargin, 1e-6);

            const OffsetAngleMargin &margin = {
                {lowerOffsetMargin, upperOffsetMargin},
                {lowerAngleMargin, upperAngleMargin}
            };
            const double meanInvRanges = invRangesFiltered.mean();

            LOG_INFO(
                "Offset increased: ", upperOffsetMargin, ", Offset decreased: ", lowerOffsetMargin,
                ", Angle increased: ", upperAngleMargin, ", Angle decreased: ", lowerAngleMargin,
                ", Mean inv ranges: ", meanInvRanges
            );

            const OffsetAngle scanlineAttributes = {.offset = fitResult->slope, .angle = fitResult->intercept};
            const ScanlineLimits &newLimits = computeScanlineLimits(
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
            .limits = state == FitConvergenceState::CONFIRMED ? std::make_optional(currentScanlineLimits) : std::nullopt,
            .success = state == FitConvergenceState::CONFIRMED,
            .ciTooWide = ciTooWide,
        };
    }

    HeuristicScanline VerticalIntrinsicsEstimator::computeHeuristicScanline(
        double invRangesMean, double phisMean
    ) const {
        std::optional<uint32_t> closestScanlineIdTop = std::nullopt;
        std::optional<uint32_t> closestScanlineIdBottom = std::nullopt;
        double closestScanlineTopDistance = std::numeric_limits<double>::infinity();
        double closestScanlineBottomDistance = std::numeric_limits<double>::infinity();

        for (const auto &[scanlineId, scanline]: scanlineInfoMap) {
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
            const auto &scanline = scanlineInfoMap.at(scanlineId);
            double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;
            offsetDiffs.emplace_back(offsetDiff);
        }

        const double maxOffsetDiff = std::ranges::max(offsetDiffs);
        double meanOffset = 0;
        for (const auto &scanlineId: validScanlineIds) {
            meanOffset += scanlineInfoMap.at(scanlineId).values.offset / static_cast<double>(validScanlineIds.size());
        }

        return HeuristicScanline{
            .offset = meanOffset,
            .offsetCi = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2},
            .dependencies = validScanlineIds
        };
    }

    ScanlineIntersectionInfo VerticalIntrinsicsEstimator::computeScanlineIntersectionInfo(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId
    ) {
        const Eigen::ArrayXi &conflictingScanlinesIdsVerbose = pointsScanlinesIds(scanline.limits.indices);
        Eigen::ArrayX<bool> empiricalIntersectionMask = Eigen::ArrayX<bool>::Constant(scanlineId + 1, false);
        bool empiricalIntersection = false;

        for (const int32_t conflictingId: conflictingScanlinesIdsVerbose) {
            if (conflictingId >= 0) {
                empiricalIntersection = true;
                empiricalIntersectionMask[conflictingId] = true;
            }
        }

        Eigen::ArrayX<bool> theoreticalIntersectionMask = Eigen::ArrayX<bool>::Constant(scanlineId + 1, true);
        const std::vector boundsPointers = {&ScanlineAngleBounds::bottom, &ScanlineAngleBounds::top};

        for (const auto thisBound: boundsPointers) {
            const double thisLower = (angleBounds.*thisBound).lower;
            const double thisUpper = (angleBounds.*thisBound).upper;

            for (const auto otherBound: boundsPointers) {
                Eigen::ArrayXd maxTheoreticalSigns = Eigen::ArrayXd::Ones(scanlineId + 1);
                Eigen::ArrayXd minTheoreticalSigns = Eigen::ArrayXd::Ones(scanlineId + 1);

                for (uint32_t otherId = 0; otherId < scanlineId + 1; ++otherId) {
                    if (!scanlineInfoMap.contains(otherId)) {
                        continue;
                    }

                    const double otherLower = (scanlineInfoMap[otherId].theoreticalAngleBounds.*otherBound).lower;
                    const double otherUpper = (scanlineInfoMap[otherId].theoreticalAngleBounds.*otherBound).upper;

                    minTheoreticalSigns[otherId] = Utils::compare(thisLower, otherLower);
                    maxTheoreticalSigns[otherId] = Utils::compare(thisUpper, otherUpper);
                }

                theoreticalIntersectionMask = theoreticalIntersectionMask && (
                                                  (maxTheoreticalSigns * minTheoreticalSigns).array() != 1).array();
            }
        }

        return {
            .empiricalIntersectionMask = std::move(empiricalIntersectionMask),
            .theoreticalIntersectionMask = std::move(theoreticalIntersectionMask),
            .empiricalIntersection = empiricalIntersection,
            .theoreticalIntersection = theoreticalIntersectionMask.any()
        };
    }

    bool VerticalIntrinsicsEstimator::performScanlineConflictResolution(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId,
        const HoughCell &houghMax
    ) {
        const ScanlineConflictsResult &conflicts = evaluateScanlineConflicts(
            angleBounds, scanline, scanlineId, houghMax
        );

        if (conflicts.shouldReject) {
            LOG_INFO("Scanline rejected");
            LOG_INFO("");

            hough->eraseByHash(houghMax.hash);

            if (conflicts.conflictingScanlines.size() > 0) {
                HashToConflictValue &hashToConflictValue = hashesToConflictsMap[houghMax.hash];

                hashToConflictValue.conflictingScanlines.insert(
                    conflicts.conflictingScanlines.begin(), conflicts.conflictingScanlines.end()
                );
                hashToConflictValue.votes = houghMax.votes;
            }

            return false;
        }

        std::queue<uint32_t> scanlinesToRemoveQueue;
        std::unordered_set<uint32_t> scanlinesToRemoveSet;

        for (const auto otherId: conflicts.conflictingScanlines) {
            if (scanlinesToRemoveSet.insert(otherId).second) {
                scanlinesToRemoveQueue.push(otherId);
            }
        }

        while (!scanlinesToRemoveQueue.empty()) {
            const uint32_t otherId = scanlinesToRemoveQueue.front();
            scanlinesToRemoveQueue.pop();

            auto dependencyRange = reverseScanlinesDependencyMap.equal_range(otherId);
            for (auto it = dependencyRange.first; it != dependencyRange.second; ++it) {
                if (scanlinesToRemoveSet.insert(it->second).second) {
                    scanlinesToRemoveQueue.push(it->second);
                }
            }
        }

        LOG_INFO("Removing scanlines: ", scanlinesToRemoveSet);

        for (const uint32_t otherId: scanlinesToRemoveSet) {
            const ScanlineInfo &otherScanline = scanlineInfoMap[otherId];
            const uint64_t conflictingHash = otherScanline.houghHash;
            const double conflictingVotes = otherScanline.houghVotes;

            hough->restoreVotes(conflictingHash, conflictingVotes);
            unassignedPoints += otherScanline.pointsCount;
            pointsScanlinesIds = (pointsScanlinesIds == otherId).select(-1, pointsScanlinesIds);

            // TODO careful with this, perhaps select
            scanlineInfoMap.erase(otherId);
            reverseScanlinesDependencyMap.erase(otherId);

            // Remove entries where scanlineId is in the value
            for (auto it = reverseScanlinesDependencyMap.begin(); it != reverseScanlinesDependencyMap.end();) {
                if (it->second == otherId) {
                    it = reverseScanlinesDependencyMap.erase(it);
                } else {
                    ++it;
                }
            }

            for (auto it = hashesToConflictsMap.begin(); it != hashesToConflictsMap.end();) {
                const uint64_t hash = it->first;
                HashToConflictValue &hashToConflictValue = it->second;

                hashToConflictValue.conflictingScanlines.erase(otherId);

                if (hashToConflictValue.conflictingScanlines.empty()) {
                    LOG_INFO("Restored hash: ", hash);

                    hough->restoreVotes(hash, hashToConflictValue.votes);
                    it = hashesToConflictsMap.erase(it);
                } else {
                    ++it;
                }
            }

            // Store the removed line, as conflicting with current one
            hashesToConflictsMap.emplace(
                conflictingHash, HashToConflictValue{
                    .conflictingScanlines = {scanlineId},
                    .votes = conflictingVotes
                }
            );

            LOG_INFO("Added hash ", conflictingHash, " to the map");
        }

        return true;
    }

    // TODO review this method, maybe precompute actually conflicting scanlines and do ifs based on that
    ScanlineConflictsResult VerticalIntrinsicsEstimator::evaluateScanlineConflicts(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId,
        const HoughCell &houghMax
    ) {
        const ScanlineIntersectionInfo &intersectionInfo = computeScanlineIntersectionInfo(
            angleBounds, scanline, scanlineId
        );

        if (!intersectionInfo.anyIntersection()) {
            return {
                .shouldReject = false,
                .conflictingScanlines = Eigen::ArrayXi()
            };
        }

        LOG_WARN("Possible problem detected");
        LOG_INFO(
            "Intersects other scanline: ", intersectionInfo.empiricalIntersection? "True": "False",
            ", Intersects theoretically: ", intersectionInfo.theoreticalIntersection,
            ", Fit success: ", "True",
            ", Points in scanline: ", scanline.limits.indices.size(), " vs ", houghMax.votes
        );


        Eigen::ArrayXi conflictingScanlines = Eigen::ArrayXi::Zero(intersectionInfo.empiricalIntersectionMask.size());
        int j = 0;
        for (int i = 0; i < intersectionInfo.empiricalIntersectionMask.size(); ++i) {
            if (intersectionInfo.anyIntersection(i)) {
                conflictingScanlines(j++) = i;
            }
        }
        conflictingScanlines.conservativeResize(j);

        Eigen::ArrayXd conflictingScanlinesUncertainties(conflictingScanlines.size());
        for (int i = 0; i < conflictingScanlines.size(); ++i) {
            conflictingScanlinesUncertainties(i) = scanlineInfoMap[conflictingScanlines(i)].uncertainty;
        }

        LOG_INFO(
            "Intersects with scanlines: ", conflictingScanlines,
            ", Current scanline uncertainty: ", scanline.uncertainty,
            ", Conflicting scanlines uncertainties: ", conflictingScanlinesUncertainties
        );

        if (scanline.limits.indices.size() == unassignedPoints) {
            LOG_WARN(
                "Warning: This is the last scanline, so we will accept it if it is "
                "not empirically intersecting with other scanlines"
            );
            // TODO This is causing problems, for example in durlar_4d_single_upper_bound_.._.._datasets_durlar_dataset_DurLAR_DurLAR_20210716_ouster_points_data_0000035800.bin/output.txt
            // TODO maybe recover last scanline flag (evaluate whether this case occurs)
            return {
                .shouldReject = intersectionInfo.empiricalIntersection,
                .conflictingScanlines = std::move(conflictingScanlines)
            };
        }

        if (scanline.uncertainty >= conflictingScanlinesUncertainties.minCoeff()) {
            if (scanline.uncertainty == std::numeric_limits<double>::infinity()) {
                LOG_INFO(
                    "New uncertainty is infinite, but so are the conflicting scanlines uncertainties. Intersects other empirical: ",
                    intersectionInfo.empiricalIntersection? "True": "False", "."
                );

                return {
                    .shouldReject = intersectionInfo.empiricalIntersection,
                    .conflictingScanlines = std::move(conflictingScanlines)
                };
            }

            // TODO thoroughly test this case both in python and C++
            LOG_INFO(
                "New uncertainty is higher than conflicting scanlines uncertainties. Rejecting current scanline"
            );

            Eigen::ArrayXi actuallyConflictingScanlines = Eigen::ArrayXi::Zero(conflictingScanlines.size());

            j = 0;
            for (int i = 0; i < conflictingScanlines.size(); ++i) {
                if (scanline.uncertainty >= conflictingScanlinesUncertainties(i)) {
                    actuallyConflictingScanlines(j++) = conflictingScanlines(i);
                }
            }
            actuallyConflictingScanlines.conservativeResize(j);

            return {
                .shouldReject = true,
                .conflictingScanlines = std::move(actuallyConflictingScanlines)
            };
        }

        LOG_INFO(
            "New uncertainty is lower than conflicting scanlines uncertainties. Rejecting conflicting scanlines"
        );

        return {
            .shouldReject = false,
            .conflictingScanlines = std::move(conflictingScanlines)
        };
    }

    // TODO remove this
    void VerticalIntrinsicsEstimator::writeToJson(const VerticalIntrinsicsResult &result) {
        const std::optional<std::string> outputPath = getOutputPath();
        if (outputPath) {
            nlohmann::json json = verticalIntrinsicsResultToJson(result);
            std::ofstream outFile(std::filesystem::path(*outputPath) / "summary.json");
            outFile << json.dump(4);
        }
    }
} // namespace accurate_ri
