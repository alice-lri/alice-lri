#include "CoarseToFineHorizontalIntrinsicsEstimator.h"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <vector>

#include "intrinsics/horizontal/offset/RansacHOffset.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

// TODO fix inconsistent naming for resolution
namespace accurate_ri {
    HorizontalIntrinsicsResult CoarseToFineHorizontalIntrinsicsEstimator::estimate(
        const PointArray &points, const VerticalIntrinsicsResult &vertical
    ) {
        HorizontalIntrinsicsResult result;
        result.scanlines.resize(vertical.scanlinesCount);

        const HorizontalScanlineArray scanlineArray(
            points, vertical.fullScanlines.pointsScanlinesIds, vertical.scanlinesCount
        );

        std::unordered_set<uint32_t> heuristicScanlines;

        for (uint32_t scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            if (scanlineArray.getSize(scanlineIdx) < 16) {
                LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than 16 points, queueing for heuristics");
                heuristicScanlines.insert(scanlineIdx);

                continue;
            }

            int32_t bestResInt = optimizeResolutionCoarse(scanlineArray, scanlineIdx);

            const ResolutionOffsetLoss optimizeResult = optimizeJoint(scanlineArray, scanlineIdx, bestResInt);
            bestResInt = optimizeResult.resolution;
            const double bestOffset = optimizeResult.offset;
            const double bestLoss = optimizeResult.loss;

            result.scanlines[scanlineIdx] = {bestResInt, bestOffset, false};

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", bestResInt, "\tOffset: ", bestOffset, "\tPoints: ",
                scanlineArray.getSize(scanlineIdx), "\tLoss: ", bestLoss
            );
        }

        if (!heuristicScanlines.empty()) {
            updateHeuristicScanlines(result.scanlines, heuristicScanlines, scanlineArray);
        }

        return result;
    }

    int32_t CoarseToFineHorizontalIntrinsicsEstimator::optimizeResolutionCoarse(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        int32_t bestResInt = -1;
        int32_t resStart = 500;
        int32_t resEnd = 50000;

        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &rangesXy = scanlineArray.getRangesXy(scanlineIdx);

        for (const int32_t step: {117, 1}) {
            double minLoss = std::numeric_limits<double>::infinity();

            for (int32_t resInt = resStart; resInt < resEnd; resInt += step) {
                const double resolution = 2 * M_PI / resInt;
                const double loss = computeCoarseLoss(thetas, rangesXy, 0, resolution);

                if (loss < minLoss) {
                    minLoss = loss;
                    bestResInt = resInt;
                }
            }

            resStart = bestResInt - step;
            resEnd = bestResInt + step;
        }

        return bestResInt;
    }

    std::pair<double, double> CoarseToFineHorizontalIntrinsicsEstimator::optimizeOffsetCoarse(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const double resolution
    ) {
        double offsetStart = -0.2;
        double offsetEnd = 0.2;
        double bestOffset = 0.0;

        double bestLoss = std::numeric_limits<double>::infinity();

        for (double step: {1e-2, 1e-4, 1e-7}) {
            double localBestOffset = bestOffset;
            double localBestLoss = std::numeric_limits<double>::infinity();

            for (double offset = offsetStart; offset < offsetEnd; offset += step) {
                const double loss = computeCoarseLoss(thetas, ranges, offset, resolution);

                if (loss < localBestLoss) {
                    localBestLoss = loss;
                    localBestOffset = offset;
                }
            }

            bestOffset = localBestOffset;
            bestLoss = localBestLoss;

            offsetStart = bestOffset - step;
            offsetEnd = bestOffset + step;
        }

        return {bestOffset, bestLoss};
    }

    ResolutionOffsetLoss CoarseToFineHorizontalIntrinsicsEstimator::optimizeJoint(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t initialResInt
    ) {
        constexpr uint32_t minDiv = 1;
        constexpr uint32_t maxDiv = 30;
        constexpr int32_t minDelta = -50; // TODO maybe derive bounds for these deltas as a function of number of points
        constexpr int32_t maxDelta = 50;

        double minLoss = std::numeric_limits<double>::infinity();
        double bestOffset = 0;
        int32_t bestCandidate = initialResInt;

        for (int32_t div = minDiv; div < maxDiv; ++div) {
            for (int32_t delta = minDelta; delta < maxDelta; ++delta) {
                const int32_t candidateMultiple = initialResInt + delta;

                if (candidateMultiple <= 0 || candidateMultiple % div != 0) {
                    continue;
                }

                const int32_t candidate = candidateMultiple / div;
                // TODO use acutal coords eps
                const std::optional<RansacHOffsetResult> rhResult = RansacHOffset::computeOffset(
                    scanlineArray, scanlineIdx, candidate
                );

                if (!rhResult.has_value()) {
                    continue;
                }

                if (rhResult->loss < minLoss) {
                    minLoss = rhResult->loss;
                    bestCandidate = candidate;
                    bestOffset = rhResult->offset;
                }
            }
        }

        return {.resolution = bestCandidate, .offset = bestOffset, .loss = minLoss};
    }

    std::pair<double, double> CoarseToFineHorizontalIntrinsicsEstimator::optimizeOffsetPrecise(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const double resolution,
        const double offsetGuess
    ) {
        double offsetStart = offsetGuess - 1e-2;
        double offsetEnd = offsetGuess + 1e-2;
        double bestOffset = 0.0;
        double bestLoss = std::numeric_limits<double>::infinity();

        for (double step: {1e-4, 1e-7}) {
            double localBestOffset = bestOffset;
            double localBestLoss = std::numeric_limits<double>::infinity();

            for (double offset = offsetStart; offset < offsetEnd; offset += step) {
                const double loss = computePreciseLoss(thetas, ranges, offset, resolution);

                if (loss < localBestLoss) {
                    localBestLoss = loss;
                    localBestOffset = offset;
                }
            }

            bestOffset = localBestOffset;
            bestLoss = localBestLoss;

            offsetStart = bestOffset - step;
            offsetEnd = bestOffset + step;
        }

        return {bestOffset, bestLoss};
    }

    double CoarseToFineHorizontalIntrinsicsEstimator::computeCoarseLoss(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const double offset,
        const double resolution
    ) {
        Eigen::ArrayXd corrected = thetas - offset / ranges;
        corrected -= corrected.minCoeff();
        std::ranges::sort(corrected);

        const Eigen::ArrayXd diffs = Utils::diff(corrected);
        const Eigen::ArrayXd idealDiffs = (diffs / resolution).round() * resolution;

        return (diffs - idealDiffs).abs().mean() / resolution;
    }

    double CoarseToFineHorizontalIntrinsicsEstimator::computePreciseLoss(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const double offset,
        const double resolution
    ) {
        Eigen::ArrayXd corrected = thetas - offset / ranges;
        corrected -= corrected.minCoeff();
        std::ranges::sort(corrected);

        const Eigen::ArrayXd aligned = (corrected / resolution).round() * resolution;

        return (corrected - aligned).abs().mean() / resolution;
    }

    void CoarseToFineHorizontalIntrinsicsEstimator::updateHeuristicScanlines(
        std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<uint32_t> &heuristicScanlines,
        const HorizontalScanlineArray &scanlineArray
    ) {
        std::vector<double> otherOffsets;
        std::unordered_set<int32_t> otherResolutions;

        for (int i = 0; i < scanlines.size(); ++i) {
            if (heuristicScanlines.contains(i)) {
                continue;
            }

            otherResolutions.insert(scanlines[i].resolution);

            const double currentOffset = scanlines[i].offset;
            double minOffsetDiff = std::numeric_limits<double>::infinity();

            for (const double otherOffset: otherOffsets) {
                const double offsetDiff = std::abs(currentOffset - otherOffset);

                if (offsetDiff < minOffsetDiff) {
                    minOffsetDiff = offsetDiff;
                }
            }

            if (minOffsetDiff > 1e-6) {
                otherOffsets.emplace_back(scanlines[i].offset);
            }
        }

        for (const uint32_t scanlineIdx: heuristicScanlines) {
            const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
            const Eigen::ArrayXd &rangesXy = scanlineArray.getRangesXy(scanlineIdx);

            const auto [bestResInt, bestOffset] = optimizeFromCandidatesPrecise(
                thetas, rangesXy, otherResolutions, otherOffsets
            );

            scanlines[scanlineIdx] = {bestResInt, bestOffset, true};

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", bestResInt, "\tOffset: ", bestOffset, "\tPoints: ",
                thetas.size()
            );
        }
    }

    std::pair<int32_t, double> CoarseToFineHorizontalIntrinsicsEstimator::optimizeFromCandidatesPrecise(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const std::unordered_set<int32_t> &candidateResInts,
        const std::vector<double> &candidateOffsets
    ) {
        int32_t bestResInt = -1;
        double bestOffset = 0;
        double bestLoss = std::numeric_limits<double>::infinity();

        for (const int32_t resInt: candidateResInts) {
            double resolution = 2 * M_PI / resInt;

            for (double offset: candidateOffsets) {
                const double loss = computePreciseLoss(thetas, ranges, offset, resolution);

                if (loss < bestLoss) {
                    bestLoss = loss;
                    bestResInt = resInt;
                    bestOffset = offset;
                }
            }
        }

        return {bestResInt, bestOffset};
    }
} // namespace accurate_ri
