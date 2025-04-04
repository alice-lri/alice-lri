#include "CoarseToFineHorizontalIntrinsicsEstimator.h"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    HorizontalIntrinsicsResult CoarseToFineHorizontalIntrinsicsEstimator::estimate(
        const PointArray &points, const VerticalIntrinsicsResult &vertical
    ) {
        HorizontalIntrinsicsResult result;
        result.scanlines.resize(vertical.scanlinesCount);

        std::vector<std::vector<int>> pointsByScanline(vertical.scanlinesCount);
        for (int i = 0; i < vertical.pointsCount; ++i) {
            const int32_t scanlineIdx = vertical.fullScanlines.pointsScanlinesIds[i];

            if (scanlineIdx >= 0) {
                pointsByScanline[scanlineIdx].push_back(i);
            }
        }

        for (int scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            const auto &indices = pointsByScanline[scanlineIdx];
            if (indices.size() < 2) { // TODO heuristic
                result.scanlines[scanlineIdx] = {0, 0.0, true};
                continue;
            }

            std::vector<std::pair<double, double>> thetaRangePairs;

            for (int idx: indices) {
                double theta = points.getTheta(idx);
                double range = points.getRange(idx);

                thetaRangePairs.emplace_back(theta, range);
            }


            std::ranges::sort(
                thetaRangePairs, [](const std::pair<double, double> &a, const std::pair<double, double> &b) {
                    return a.first < b.first;
                }
            );

            Eigen::ArrayXd thetas = Eigen::ArrayXd(thetaRangePairs.size());
            Eigen::ArrayXd ranges = Eigen::ArrayXd(thetaRangePairs.size());

            for (int i = 0; i < thetaRangePairs.size(); ++i) {
                thetas(i) = thetaRangePairs[i].first;
                ranges(i) = thetaRangePairs[i].second;
            }

            thetas = thetas - thetas.minCoeff();

            int32_t bestResInt = optimizeResolutionCoarse(thetas, ranges);
            double bestResolution = 2 * M_PI / bestResInt;
            auto offsetPairCoarse = optimizeOffsetCoarse(thetas, ranges, bestResolution);
            double bestOffset = offsetPairCoarse.first;

            bestResInt = refineResolutionPrecise(thetas, ranges, bestResInt, bestOffset);
            bestResolution = 2 * M_PI / bestResInt;
            auto offsetPairPrecise = optimizeOffsetPrecise(thetas, ranges, bestResolution);
            bestOffset = offsetPairPrecise.first;
            double bestLoss = offsetPairPrecise.second;

            result.scanlines[scanlineIdx] = {bestResInt, bestOffset, false};

            LOG_INFO(
                "Scanline ID: ", scanlineIdx, "\tRes: ", bestResInt, "\tOffset: ", bestOffset, "\tPoints: ",
                thetas.size(), "\tLoss: ", bestLoss
            );
        }

        return result;
    }

    int32_t CoarseToFineHorizontalIntrinsicsEstimator::optimizeResolutionCoarse(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges
    ) {
        int32_t bestResInt = -1;
        int32_t resStart = 500;
        int32_t resEnd = 50000;

        for (const int32_t step: {117, 1}) {
            double minLoss = std::numeric_limits<double>::infinity();

            for (int32_t resInt = resStart; resInt < resEnd; resInt += step) {
                const double resolution = 2 * M_PI / resInt;
                const double loss = computeCoarseLoss(thetas, ranges, 0, resolution);

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

    int32_t CoarseToFineHorizontalIntrinsicsEstimator::refineResolutionPrecise(
        const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges,
        const int32_t initialResInt,
        const double offset
    ) {
        const uint32_t minDiv = 1;
        const uint32_t maxDiv = 30;
        const int32_t minDelta = -50;
        const int32_t maxDelta = 50;

        double minLoss = std::numeric_limits<double>::infinity();
        int32_t bestCandidate = initialResInt;

        for (uint32_t div = minDiv; div < maxDiv; ++div) {
            for (int32_t delta = minDelta; delta < maxDelta; ++delta) {
                const int64_t candidateMultiple = initialResInt + delta;

                if (candidateMultiple <= 0 || candidateMultiple % div != 0) {
                    continue;
                }

                const uint32_t candidate = candidateMultiple / div;
                const double resolution = 2 * M_PI / candidate;
                const double loss = computePreciseLoss(thetas, ranges, offset, resolution);

                if (loss < minLoss) {
                    minLoss = loss;
                    bestCandidate = candidate;
                }
            }
        }

        return bestCandidate;
    }

    std::pair<double, double> CoarseToFineHorizontalIntrinsicsEstimator::optimizeOffsetPrecise(
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
} // namespace accurate_ri
