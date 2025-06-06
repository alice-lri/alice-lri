#include "HorizontalIntrinsicsEstimator.h"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <vector>

#include "Constants.h"
#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "intrinsics/horizontal/helper/SegmentedMedianSlopeEstimator.h"
#include "intrinsics/horizontal/multiline/PeriodicMultilineFitter.h"
#include "intrinsics/horizontal/resolution/MadResolutionLoss.h"
#include "math/Stats.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

// TODO fix inconsistent naming for resolution
namespace accurate_ri {
    HorizontalIntrinsicsResult HorizontalIntrinsicsEstimator::estimate(
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
            std::optional<ScanlineHorizontalInfo> info = estimateScanline(scanlineArray, scanlineIdx);

            if (info) {
                result.scanlines[scanlineIdx] = *info;
            } else {
                heuristicScanlines.insert(scanlineIdx);
            }
        }

        if (!heuristicScanlines.empty()) {
            updateHeuristicScanlines(result.scanlines, heuristicScanlines, scanlineArray);
        }

        return result;
    }

    std::optional<ScanlineHorizontalInfo> HorizontalIntrinsicsEstimator::estimateScanline(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        LOG_DEBUG("Processing horizontal scanline: ", scanlineIdx);

        if (scanlineArray.getSize(scanlineIdx) < 16) {
            LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than 16 points, queueing for heuristics");
            return std::nullopt;
        }

        int32_t bestResInt = madOptimalResolution(scanlineArray, scanlineIdx);
        const auto optimizeResult = optimizeJoint(scanlineArray, scanlineIdx, bestResInt);

        if (!optimizeResult) {
            LOG_ERROR("Horizontal optimization failed for scanline ", scanlineIdx);
            return std::nullopt;
        }

        bestResInt = optimizeResult->resolution;
        const double bestOffset = optimizeResult->offset;
        const double bestLoss = optimizeResult->loss;

        LOG_INFO(
            "Scanline ID: ", scanlineIdx, "\tRes: ", bestResInt, "\tOffset: ", bestOffset, "\tPoints: ",
            scanlineArray.getSize(scanlineIdx), "\tLoss: ", bestLoss
        );

        return std::make_optional<ScanlineHorizontalInfo>(
            {
                .resolution = bestResInt,
                .offset = bestOffset,
                .heuristic = false
            }
        );
    }

    std::optional<ResolutionOffsetLoss> HorizontalIntrinsicsEstimator::optimizeJoint(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t initialResolution
    ) {
        const int32_t scanlineSize = scanlineArray.getSize(scanlineIdx);
        const std::vector<int32_t> resolutions = generateCandidateResolutions(initialResolution, scanlineSize);

        std::optional<ResolutionOffsetLoss> bestCandidate;

        for (const int32_t resolution: resolutions) {
            const ResolutionOffsetLoss candidate = optimizeJointCandidateResolution(
                scanlineArray, scanlineIdx, resolution
            );

            if (!bestCandidate || candidate.loss < bestCandidate->loss) {
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    std::vector<int32_t> HorizontalIntrinsicsEstimator::generateCandidateResolutions(
        const int32_t initialResolution, const int32_t scanlineSize
    ) {
        std::vector<int32_t> resolutions;

        for (int32_t i = 0; i < 2 * Constant::MAX_RESOLUTION_DELTA + 1; ++i) {
            const int32_t delta = i % 2 == 0 ? -i / 2 : (i + 1) / 2;
            const int32_t candidateResolutionMult = initialResolution + delta;

            if (candidateResolutionMult < 0) {
                continue;
            }

            const int32_t maxDiv = candidateResolutionMult / scanlineSize;

            for (int32_t divisor = 1; divisor <= maxDiv; ++divisor) {
                if (candidateResolutionMult % divisor == 0) {
                    resolutions.emplace_back(candidateResolutionMult / divisor);
                }
            }
        }

        return resolutions;
    }

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::optimizeJointCandidateResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t resolution
    ) {
        LOG_DEBUG("Candidate resolution: ", resolution);

        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const auto diffToIdeal = HorizontalMath::computeDiffToIdeal(
            scanlineArray.getThetas(scanlineIdx), resolution, true
        );

        const SegmentedMedianSlopeEstimator slopeEstimator(Constant::INV_RANGES_BREAK_THRESHOLD, 0.5);
        const double offsetGuess = slopeEstimator.estimateSlope(invRangesXy, diffToIdeal);
        LOG_DEBUG("Offset guess: ", offsetGuess);

        PeriodicMultilineFitter fitter(resolution);
        const PeriodicMultilineFitResult fitResult = fitter.fit(scanlineArray, scanlineIdx, offsetGuess);

        return ResolutionOffsetLoss(resolution, fitResult.model.slope, fitResult.loss);
    }

    void HorizontalIntrinsicsEstimator::updateHeuristicScanlines(
        std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
        const HorizontalScanlineArray &scanlineArray
    ) {
        const std::unordered_set<double> otherOffsets = getUniqueOffsets(scanlines, heuristicScanlines);
        const std::unordered_set<int32_t> otherResolutions = getUniqueResolutions(scanlines, heuristicScanlines);

        for (const int32_t scanlineIdx: heuristicScanlines) {
            const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
            const Eigen::ArrayXd &rangesXy = scanlineArray.getRangesXy(scanlineIdx);

            const ResolutionOffsetLoss bestParams = optimizeFromCandidatesPrecise(
                thetas, rangesXy, otherResolutions, otherOffsets
            );

            scanlines[scanlineIdx] = ScanlineHorizontalInfo{bestParams.resolution, bestParams.offset, true};

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", bestParams.resolution, "\tOffset: ", bestParams.offset,
                "\tPoints: ", thetas.size()
            );
        }
    }

    std::unordered_set<int32_t> HorizontalIntrinsicsEstimator::getUniqueResolutions(
        std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
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
        std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
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

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::optimizeFromCandidatesPrecise(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const std::unordered_set<int32_t> &candidateResolutions,
        const std::unordered_set<double> &candidateOffsets
    ) {
        ResolutionOffsetLoss bestCandidate(0, 0, std::numeric_limits<double>::infinity());

        for (const int32_t resolution: candidateResolutions) {
            const double thetaStep = 2 * M_PI / resolution;

            for (const double offset: candidateOffsets) {
                const double loss = computePreciseLoss(thetas, ranges, offset, thetaStep);

                if (loss < bestCandidate.loss) {
                    bestCandidate.resolution = resolution;
                    bestCandidate.offset = offset;
                    bestCandidate.loss = loss;
                }
            }
        }

        return bestCandidate;
    }

    double HorizontalIntrinsicsEstimator::computePreciseLoss(
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

    int32_t HorizontalIntrinsicsEstimator::madOptimalResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        // TODO do not hardcode these
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const int32_t minResolution = invRangesXy.innerSize();

        double minLoss = std::numeric_limits<double>::infinity();
        int32_t optimalResolution = -1;

        for (int32_t resolution = minResolution; resolution < Constant::MAX_RESOLUTION; ++resolution) {
            const double loss = MadResolutionLoss::computeResolutionLoss(invRangesXy, thetas, resolution);

            if (loss < minLoss) {
                minLoss = loss;
                optimalResolution = resolution;
            }
        }

        return optimalResolution;
    }
} // namespace accurate_ri
