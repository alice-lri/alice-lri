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
#include "plotty/matplotlibcpp.hpp"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

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
            const std::optional<ScanlineHorizontalInfo> info = estimateScanline(scanlineArray, scanlineIdx);

            if (info) {
                result.scanlines[scanlineIdx] = *info;
            } else {
                heuristicScanlines.insert(scanlineIdx);
                LOG_INFO("Scanline ", scanlineIdx, " will be estimated heuristically");
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

        if (scanlineArray.getSize(scanlineIdx) < Constant::HORIZONTAL_MIN_POINTS_PER_SCANLINE) {
            LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than the minimum points");
            return std::nullopt;
        }

        //int32_t bestResolution = madOptimalResolution(scanlineArray, scanlineIdx);
        int32_t bestResolution = 0;
        const auto optimizeResult = optimizeJoint(scanlineArray, scanlineIdx, bestResolution);

        if (!optimizeResult) {
            LOG_WARN("Horizontal optimization failed for scanline ", scanlineIdx);
            return std::nullopt;
        }

        bestResolution = optimizeResult->resolution;
        const double bestOffset = optimizeResult->offset;
        const double bestLoss = optimizeResult->loss;

        LOG_INFO(
            "Scanline ID: ", scanlineIdx, "\tRes: ", bestResolution, "\tOffset: ", bestOffset,
            "\tTheta Offset: ", optimizeResult->thetaOffset, "\tPoints: ", scanlineArray.getSize(scanlineIdx),
            "\tLoss: ", bestLoss
        );

        return std::make_optional<ScanlineHorizontalInfo>(
            {
                .resolution = bestResolution,
                .offset = bestOffset,
                .thetaOffset = optimizeResult->thetaOffset,
                .heuristic = false
            }
        );
    }

    // TODO this is not optional anymore
    std::optional<ResolutionOffsetLoss> HorizontalIntrinsicsEstimator::optimizeJoint(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t initialResolution
    ) {
        const int32_t scanlineSize = scanlineArray.getSize(scanlineIdx);
        std::vector<int32_t> resolutions;
        if (initialResolution != 0) {
            resolutions = generateCandidateResolutions(initialResolution, scanlineSize);
        } else {
            resolutions = generateCandidateResolutionsMad(scanlineArray, scanlineIdx);
        }

        std::optional<ResolutionOffsetLoss> bestCandidate = std::nullopt;

        for (const int32_t resolution: resolutions) {
            const ResolutionOffsetLoss candidate = optimizeJointCandidateResolution(
                scanlineArray, scanlineIdx, resolution
            );

            LOG_DEBUG(
                "Candidate resolution: ", candidate.resolution, ", offset: ", candidate.offset,
                "theta offset: ", candidate.thetaOffset, ", loss: ", candidate.loss
            );

            if (std::abs(candidate.offset) > Constant::MAX_OFFSET) {
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

    std::vector<int32_t> HorizontalIntrinsicsEstimator::generateCandidateResolutionsMad(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const int32_t minResolution = invRangesXy.innerSize();

        Eigen::ArrayXd mads(Constant::MAX_RESOLUTION - minResolution + 1);

        for (int32_t resolution = minResolution; resolution <= Constant::MAX_RESOLUTION; ++resolution) {
            const double loss = MadResolutionLoss::computeResolutionLoss(invRangesXy, thetas, resolution);
            mads(resolution - minResolution) = loss;
        }

        mads /= mads.maxCoeff();

        std::vector<int32_t> resolutions;
        resolutions.reserve(250);
        for (int32_t resolution = minResolution; resolution < Constant::MAX_RESOLUTION; ++resolution) {
            if (mads(resolution - minResolution) < 0.5) { // TODO extract constant
                resolutions.emplace_back(resolution);
            }
        }

        return resolutions;
    }

    ResolutionOffsetLoss HorizontalIntrinsicsEstimator::optimizeJointCandidateResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t resolution
    ) {
        LOG_DEBUG("Candidate resolution: ", resolution);

        const double thetaStep = 2 * M_PI / resolution;
        const Eigen::ArrayXd &invRangesXy = scanlineArray.getInvRangesXy(scanlineIdx);
        const auto diffToIdeal = HorizontalMath::computeDiffToIdeal(
            scanlineArray.getThetas(scanlineIdx), resolution, false
        );

        const SegmentedMedianSlopeEstimator slopeEstimator(
            Constant::INV_RANGES_BREAK_THRESHOLD, thetaStep / 4, Constant::MAX_OFFSET, thetaStep
        );

        const Stats::LRResult lrGuess = slopeEstimator.estimateSlope(invRangesXy, diffToIdeal);

        LOG_DEBUG("Slope guess: ", lrGuess.slope, ", Intercept guess: ", lrGuess.intercept);

        PeriodicMultilineFitter fitter(resolution);
        const PeriodicMultilineFitResult fitResult = fitter.fit(scanlineArray, scanlineIdx, lrGuess);

        return ResolutionOffsetLoss(
            resolution,
            fitResult.model.slope,
            Utils::positiveFmod(fitResult.model.intercept, thetaStep),
            fitResult.loss * resolution
        );
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

            const ResolutionOffsetLoss bestParams = optimizeFromCandidatesHeuristic(
                thetas, rangesXy, otherResolutions, otherOffsets
            );

            scanlines[scanlineIdx] = ScanlineHorizontalInfo{
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

        return ResolutionOffsetLoss(resolution, offset, thetaOffset, loss);
    }

    int32_t HorizontalIntrinsicsEstimator::madOptimalResolution(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
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
