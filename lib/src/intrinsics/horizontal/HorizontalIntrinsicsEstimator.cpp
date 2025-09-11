#include "HorizontalIntrinsicsEstimator.h"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>

#include "Constants.h"
#include "BuildOptions.h"
#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "intrinsics/horizontal/helper/SegmentedMedianLinearRegressor.h"
#include "helper/PeriodicFitter.h"
#include "intrinsics/horizontal/HorizontalIntrinsicsStructs.h"
#include "math/Stats.h"
#include "plotty/matplotlibcpp.hpp"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

namespace accurate_ri {
    HorizontalIntrinsicsEstimation HorizontalIntrinsicsEstimator::estimate(
        const PointArray &points, const VerticalIntrinsicsEstimation &vertical
    ) {
        PROFILE_SCOPE("HorizontalIntrinsicsEstimator::estimate");
        HorizontalIntrinsicsEstimation result;
        const int32_t scanlinesCount = vertical.scanlinesAssignations.scanlines.size();
        result.scanlines.resize(scanlinesCount);

        const HorizontalScanlineArray scanlineArray(
            points, vertical.scanlinesAssignations.pointsScanlinesIds, scanlinesCount, SortingCriteria::RANGES_XY
        );

        std::unordered_set<int32_t> heuristicScanlines;
        for (int32_t scanlineIdx = 0; scanlineIdx < scanlinesCount; ++scanlineIdx) {
            const std::optional<HorizontalScanline> info = estimateScanline(scanlineArray, scanlineIdx);

            if (info) {
                result.scanlines[scanlineIdx] = *info;
            } else {
                heuristicScanlines.insert(scanlineIdx);
                LOG_INFO("Scanline ", scanlineIdx, " will be estimated heuristically");
            }
        }

        if (heuristicScanlines.empty()) {
            return result;
        }

        if (BuildOptions::USE_HORIZONTAL_HEURISTICS) {
            updateHeuristicScanlines(result.scanlines, heuristicScanlines, scanlineArray);
        } else {
            updateBasicScanlines(result.scanlines, heuristicScanlines, scanlineArray);
        }

        return result;
    }

    std::optional<HorizontalScanline> HorizontalIntrinsicsEstimator::estimateScanline(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        LOG_DEBUG("Processing horizontal scanline: ", scanlineIdx);

        if (scanlineArray.getSize(scanlineIdx) < Constant::HORIZONTAL_MIN_POINTS_PER_SCANLINE) {
            LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than the minimum points");

            if constexpr (BuildOptions::USE_HORIZONTAL_HEURISTICS) {
                return std::nullopt;
            }

            if (scanlineArray.getSize(scanlineIdx) < 3) {
                return std::nullopt;
            }
        }

        const auto optimizeResult = findOptimalHorizontalParameters(scanlineArray, scanlineIdx);

        if (!optimizeResult) {
            LOG_WARN("Horizontal optimization failed for scanline ", scanlineIdx);
            return std::nullopt;
        }

        int32_t bestResolution = optimizeResult->resolution;
        const double bestOffset = optimizeResult->offset;
        const double bestLoss = optimizeResult->loss;

        LOG_INFO(
            "Scanline ID: ", scanlineIdx, "\tRes: ", bestResolution, "\tOffset: ", bestOffset,
            "\tTheta Offset: ", optimizeResult->thetaOffset, "\tPoints: ", scanlineArray.getSize(scanlineIdx),
            "\tLoss: ", bestLoss
        );

        return std::make_optional<HorizontalScanline>(
            {
                .resolution = bestResolution,
                .offset = bestOffset,
                .thetaOffset = optimizeResult->thetaOffset,
                .heuristic = false
            }
        );
    }

    std::optional<ResolutionOffsetLoss> HorizontalIntrinsicsEstimator::findOptimalHorizontalParameters(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        const int32_t scanlineSize = scanlineArray.getSize(scanlineIdx);
        std::optional<ResolutionOffsetLoss> bestCandidate = std::nullopt;

        for (int32_t resolution = scanlineSize; resolution <= Constant::MAX_RESOLUTION; ++resolution) {
            const ResolutionOffsetLoss candidate = optimizeJointCandidateResolution(
                scanlineArray, scanlineIdx, resolution
            );

            LOG_DEBUG(
                "Candidate resolution: ", candidate.resolution, ", offset: ", candidate.offset,
                "theta offset: ", candidate.thetaOffset, ", loss: ", candidate.loss
            );

            if (std::abs(candidate.offset) > Constant::MAX_OFFSET || !std::isfinite(candidate.offset)) {
                continue;
            }

            if (!bestCandidate || candidate.loss < bestCandidate->loss) {
                bestCandidate = candidate;
                LOG_DEBUG(
                    "New best resolution: ", candidate.resolution, ", offset: ", candidate.offset,
                    "theta offset: ", candidate.thetaOffset, ", loss: ", candidate.loss
                );
            }
        }

        return bestCandidate;
    }

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::optimizeJointCandidateResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t resolution
    ) {
        LOG_DEBUG("Candidate resolution: ", resolution);

        const double thetaStep = 2 * M_PI / resolution;
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);

        const Eigen::ArrayXd diffToIdeal = HorizontalMath::computeDiffToIdeal(
            scanlineArray.getThetas(scanlineIdx), resolution, false
        );
        const Eigen::ArrayXd diffToIdealReconstructed = HorizontalMath::computeDiffToIdeal(
            scanlineArray.getThetas(scanlineIdx), resolution, true
        );

        const SegmentedMedianLinearRegressor segmentedRegressor(
            Constant::INV_RANGES_SEGMENT_THRESHOLD, thetaStep / 4, Constant::MAX_OFFSET, thetaStep
        );
        const LRResult lrGuess = segmentedRegressor.fit(invRangesXy, diffToIdealReconstructed);

        LOG_DEBUG("Slope guess: ", lrGuess.slope, ", Intercept guess: ", lrGuess.intercept);
        const LRResult fitResult = PeriodicFitter::fit(invRangesXy, diffToIdeal, thetaStep, lrGuess.slope);

        return ResolutionOffsetLoss(
            resolution,
            fitResult.slope,
            Utils::positiveFmod(fitResult.intercept, thetaStep),
            *fitResult.mse * resolution * resolution
        );
    }

    void HorizontalIntrinsicsEstimator::updateHeuristicScanlines(
        std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
        const HorizontalScanlineArray &scanlineArray
    ) {
        const std::unordered_set<int32_t> otherResolutions = getUniqueResolutions(scanlines, heuristicScanlines);
        const std::unordered_set<double> otherOffsets = getUniqueOffsets(scanlines, heuristicScanlines);

        for (const int32_t scanlineIdx: heuristicScanlines) {
            const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
            const Eigen::ArrayXd &rangesXy = scanlineArray.getRangesXy(scanlineIdx);

            const ResolutionOffsetLoss bestParams = optimizeFromCandidatesHeuristic(
                thetas, rangesXy, otherResolutions, otherOffsets
            );

            scanlines[scanlineIdx] = HorizontalScanline{
                .resolution = bestParams.resolution,
                .offset = bestParams.offset,
                .thetaOffset = bestParams.thetaOffset,
                .heuristic = true
            };

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", bestParams.resolution, "\tOffset: ", bestParams.offset,
                "\tTheta Offset: ", bestParams.thetaOffset, "\tPoints: ", thetas.size()
            );
        }
    }

    std::unordered_set<int32_t> HorizontalIntrinsicsEstimator::getUniqueResolutions(
        const std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
    ) {
        std::unordered_set<int32_t> otherResolutions;

        for (int i = 0; i < scanlines.size(); ++i) {
            if (heuristicScanlines.contains(i)) {
                continue;
            }

            otherResolutions.insert(scanlines[i].resolution);
        }

        return otherResolutions;
    }

    std::unordered_set<double> HorizontalIntrinsicsEstimator::getUniqueOffsets(
        const std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
    ) {
        std::unordered_set<double> otherOffsets;

        for (int i = 0; i < scanlines.size(); ++i) {
            if (heuristicScanlines.contains(i)) {
                continue;
            }

            const double currentOffset = scanlines[i].offset;
            double minOffsetDiff = std::numeric_limits<double>::infinity();

            for (const double otherOffset: otherOffsets) {
                const double offsetDiff = std::abs(currentOffset - otherOffset);

                if (offsetDiff < minOffsetDiff) {
                    minOffsetDiff = offsetDiff;
                }
            }

            if (minOffsetDiff > 1e-6) {
                otherOffsets.insert(scanlines[i].offset);
            }
        }

        return otherOffsets;
    }

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::optimizeFromCandidatesHeuristic(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const std::unordered_set<int32_t> &candidateResolutions,
        const std::unordered_set<double> &candidateOffsets
    ) {
        ResolutionOffsetLoss bestCandidate(0, 0, 0, std::numeric_limits<double>::infinity());

        for (const int32_t resolution: candidateResolutions) {
            for (const double offset: candidateOffsets) {
                const ResolutionOffsetLoss candidate = computeHeuristicValues(thetas, ranges, resolution, offset);

                if (candidate.loss < bestCandidate.loss) {
                    bestCandidate = candidate;
                }
            }
        }

        return bestCandidate;
    }

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::computeHeuristicValues(
        const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, const int32_t resolution, const double offset
    ) {
        const double thetaStep = 2 * M_PI / resolution;
        Eigen::ArrayXd correctedThetas = thetas - offset / ranges;
        Utils::positiveFmodInplace(correctedThetas, 2 * M_PI);

        const Eigen::ArrayXd idealThetas = (correctedThetas / thetaStep).floor() * thetaStep;
        const Eigen::ArrayXd diffToIdeal = correctedThetas - idealThetas;
        const double thetaOffset = diffToIdeal.mean();
        const double loss = (diffToIdeal - thetaOffset).abs().mean();

        return ResolutionOffsetLoss(resolution, offset, thetaOffset, loss * resolution);
    }

    void HorizontalIntrinsicsEstimator::updateBasicScanlines(
        std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
        const HorizontalScanlineArray &scanlineArray
    ) {
        for (const int32_t scanlineIdx: heuristicScanlines) {
            scanlines[scanlineIdx] = HorizontalScanline{
                .resolution = scanlineArray.getSize(scanlineIdx),
                .offset = 0,
                .thetaOffset = 0,
                .heuristic = false
            };

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", scanlines[scanlineIdx].resolution, "\tOffset: ", 0,
                "\tTheta Offset: ", 0, "\tPoints: ", scanlineArray.getSize(scanlineIdx)
            );
        }
    }
} // namespace accurate_ri
