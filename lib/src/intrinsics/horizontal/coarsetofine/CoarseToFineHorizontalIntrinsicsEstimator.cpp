#include "CoarseToFineHorizontalIntrinsicsEstimator.h"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <vector>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "intrinsics/horizontal/offset/RansacHOffset.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "intrinsics/horizontal/resolution/MadResolutionLoss.h"
#include "math/Stats.h"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

// TODO fix inconsistent naming for resolution
namespace accurate_ri {
    HorizontalIntrinsicsResult CoarseToFineHorizontalIntrinsicsEstimator::estimate(
        const PointArray &points, const VerticalIntrinsicsResult &vertical
    ) {
        PROFILE_SCOPE("CoarseToFineHorizontalIntrinsicsEstimator::estimate");
        HorizontalIntrinsicsResult result;
        result.scanlines.resize(vertical.scanlinesCount);

        const HorizontalScanlineArray scanlineArray(
            points, vertical.fullScanlines.pointsScanlinesIds, vertical.scanlinesCount, SortingCriteria::RANGES_XY
        );

        std::unordered_set<int32_t> heuristicScanlines;

        for (int32_t scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            LOG_DEBUG("Processing horizontal scanline: ", scanlineIdx);

            if (scanlineArray.getSize(scanlineIdx) < 16) {
                LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than 16 points, queueing for heuristics");
                heuristicScanlines.insert(scanlineIdx);

                continue;
            }

            // int32_t bestResInt = optimizeResolutionCoarse(scanlineArray, scanlineIdx);
            int32_t bestResInt = madOptimalResolution(scanlineArray, scanlineIdx);
            const auto optimizeResult = optimizeJoint(scanlineArray, scanlineIdx, bestResInt);

            if (!optimizeResult) {
                LOG_ERROR("Horizontal optimization failed for scanline ", scanlineIdx);
                heuristicScanlines.insert(scanlineIdx);

                continue;
            }

            bestResInt = optimizeResult->resolution;
            const double bestOffset = optimizeResult->offset;
            const double bestLoss = optimizeResult->loss;

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

    std::optional<ResolutionOffsetLoss> CoarseToFineHorizontalIntrinsicsEstimator::optimizeJoint(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t initialResInt
    ) {
        constexpr int32_t maxDelta = 50;
        // TODO infer this more smartly, use gaussian thingies or something to derive bounds w.r.t to points

        double minLoss = std::numeric_limits<double>::infinity();
        double bestOffset = 0;
        int32_t bestResolution = initialResInt;


        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &rangesXy = scanlineArray.getRangesXy(scanlineIdx);
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const auto diffInvRangesXy = Utils::diff(invRangesXy);

        for (int32_t i = 0; i < 2 * maxDelta + 1; ++i) {
            const int32_t delta = i % 2 == 0 ? -i / 2 : (i + 1) / 2;
            const int32_t candidateResolutionMult = initialResInt + delta;

            if (candidateResolutionMult < 0) {
                continue;
            }

            const int32_t maxDiv = candidateResolutionMult / scanlineArray.getSize(scanlineIdx);

            for (int32_t divisor = 1; divisor <= maxDiv; ++divisor) {
                if (candidateResolutionMult % divisor != 0) {
                    continue;
                }

                const int32_t candidateResolution = candidateResolutionMult / divisor;

                LOG_DEBUG("Candidate resolution: ", candidateResolution);

                const auto diffToIdeal = HorizontalMath::computeDiffToIdeal(
                    scanlineArray.getThetas(scanlineIdx), candidateResolution, true
                );

                const Eigen::ArrayX<bool> nonJumpMask = diffInvRangesXy.abs() < 1e-2;
                // TODO derive this more elegantly, assuming a max offset or something
                const auto nonJumpIndices = Utils::eigenMaskToIndices(nonJumpMask);

                const double offsetGuess = computeWeightedAverageSlope(diffToIdeal, invRangesXy, nonJumpMask);
                LOG_DEBUG("Offset guess: ", offsetGuess);

                const std::optional<RansacHOffsetResult> rhResult = RansacHOffset::computeOffset(
                    scanlineArray, scanlineIdx, candidateResolution, offsetGuess
                );

                if (!rhResult.has_value()) {
                    continue;
                }

                const double resolutionDouble = 2 * M_PI / candidateResolution;
                const double loss = computePreciseLoss(thetas, rangesXy, rhResult->offset, resolutionDouble);

                if (loss < minLoss) {
                    minLoss = loss;
                    bestResolution = candidateResolution;
                    bestOffset = rhResult->offset;
                }
            }
        }

        if (minLoss == std::numeric_limits<double>::infinity()) {
            return std::nullopt;
        }

        return std::make_optional<ResolutionOffsetLoss>(
            {
                .resolution = bestResolution,
                .offset = bestOffset,
                .loss = minLoss
            }
        );
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
        std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
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

        for (const int32_t scanlineIdx: heuristicScanlines) {
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

    double CoarseToFineHorizontalIntrinsicsEstimator::computeWeightedAverageSlope(
        const Eigen::ArrayXd &diffToIdeal,
        const Eigen::ArrayXd &invRangesXy,
        const Eigen::ArrayX<bool> &nonJumpMask
    ) const {
        std::vector<std::vector<double>> diffToIdealBlocks;
        std::vector<std::vector<double>> invRangesBlocks;
        std::vector<int32_t> blockSizes;

        constexpr int expectedBlocks = 8;
        diffToIdealBlocks.reserve(expectedBlocks);
        invRangesBlocks.reserve(expectedBlocks);
        blockSizes.reserve(expectedBlocks);

        for (int i = 0; i < diffToIdeal.size(); ++i) {
            if (i == 0 || !nonJumpMask[i - 1]) {
                diffToIdealBlocks.emplace_back();
                invRangesBlocks.emplace_back();
                blockSizes.emplace_back();

                diffToIdealBlocks.back().reserve(diffToIdeal.size() / expectedBlocks);
                invRangesBlocks.back().reserve(diffToIdeal.size() / expectedBlocks);
            } else {
                diffToIdealBlocks.back().emplace_back(diffToIdeal[i]);
                invRangesBlocks.back().emplace_back(invRangesXy[i]);
                blockSizes.back()++;
            }
        }

        std::vector<double> slopes;
        std::vector<int32_t> weights;
        int32_t totalWeight = 0;

        slopes.reserve(blockSizes.size());
        weights.reserve(blockSizes.size());
        for (int32_t i = 0; i < blockSizes.size(); ++i) {
            if (blockSizes[i] < 2) {
                continue;
            }

            const Eigen::ArrayXd fitX = Eigen::Map<Eigen::ArrayXd>(invRangesBlocks[i].data(), invRangesBlocks[i].size());
            const Eigen::ArrayXd fitY = Eigen::Map<Eigen::ArrayXd>(diffToIdealBlocks[i].data(), diffToIdealBlocks[i].size());

            const double slope = Stats::simpleLinearRegression(fitX, fitY).slope;

            if (std::abs(slope) > 0.5) {
                continue;
            }

            slopes.emplace_back(slope);
            weights.emplace_back(blockSizes[i]);
            totalWeight += blockSizes[i];

            LOG_DEBUG("Using slope ", slope, " for block ", i, " with size ", blockSizes[i]);
        }

        if (slopes.empty()) {
            return 0;
        }

        double meanSlope = 0;
        for (int32_t i = 0; i < slopes.size(); ++i) {
            meanSlope += slopes[i] * weights[i];
        }
        meanSlope /= totalWeight;

        return meanSlope;
    }

    int32_t CoarseToFineHorizontalIntrinsicsEstimator::madOptimalResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        // TODO do not hardcode these
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const int32_t minResolution = invRangesXy.innerSize();
        constexpr int32_t maxResolution = 10000;

        double minLoss = std::numeric_limits<double>::infinity();
        int32_t optimalResolution = -1;

        for (int32_t resolution = minResolution; resolution < maxResolution; ++resolution) {
            const double loss = MadResolutionLoss::computeResolutionLoss(invRangesXy, thetas, resolution);

            if (loss < minLoss) {
                minLoss = loss;
                optimalResolution = resolution;
            }
        }

        return optimalResolution;
    }
} // namespace accurate_ri
