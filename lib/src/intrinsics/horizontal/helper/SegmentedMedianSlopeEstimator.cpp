#include "SegmentedMedianSlopeEstimator.h"

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "math/Stats.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    Stats::LRResult SegmentedMedianSlopeEstimator::estimateSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const {
        SlopesWeights slopeWeights;
        const int32_t n = static_cast<int32_t>(x.size());
        const Eigen::ArrayX<bool> continuityMask =
               (Utils::diff(x).abs() < segmentThresholdX) && (Utils::diff(y).abs() < segmentThresholdY);

        int blockStart = 0;
        for (int i = 1; i < n; ++i) {
            if (!continuityMask[i - 1]) {
                processBlock(x, y, blockStart + 1, i, slopeWeights);
                blockStart = i;
            }
        }

        processBlock(x, y, blockStart + 1, n, slopeWeights);

        if (slopeWeights.count() == 0) {
            return Stats::LRResult(0,0);
        }

        return Stats::LRResult(
            Stats::weightedMedian(slopeWeights.slopes, slopeWeights.weights),
            Stats::weightedMedian(slopeWeights.intercepts, slopeWeights.weights),
            Stats::mean(slopeWeights.rmses) // TODO maybe only slope needed
        );
    }

    double SegmentedMedianSlopeEstimator::computeResolutionLoss(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &thetas, const uint32_t resolution
    ) const {
        const Eigen::ArrayXd y = HorizontalMath::computeDiffToIdeal(thetas, resolution, true);
        SlopesWeights slopeWeights;
        const int32_t n = static_cast<int32_t>(x.size());
        const Eigen::ArrayX<bool> continuityMask =
               (Utils::diff(x).abs() < segmentThresholdX) && (Utils::diff(y).abs() < segmentThresholdY);

        int blockStart = 0;
        for (int i = 1; i < n; ++i) {
            if (!continuityMask[i - 1]) {
                processBlock(x, y, blockStart + 1, i, slopeWeights);
                blockStart = i;
            }
        }

        processBlock(x, y, blockStart + 1, n, slopeWeights);

        if (slopeWeights.count() == 0) {
            return std::numeric_limits<double>::infinity();
        }

        const double median = Stats::weightedMedian(slopeWeights.slopes, slopeWeights.weights);
        const Eigen::Map<const Eigen::ArrayXd> slopes(slopeWeights.slopes.data(), slopeWeights.slopes.size());
        const double loss = Stats::weightedMean((slopes - median).abs(), slopeWeights.weights);

        return loss;
    }


    void SegmentedMedianSlopeEstimator::processBlock(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const int32_t startIdx, const int32_t endIdx, SlopesWeights& slopeWeights
    ) const {
        const int size = endIdx - startIdx;
        if (size <= 2) {
            return;
        }

        const Eigen::Map<const Eigen::ArrayXd> fitX(x.data() + startIdx, size);
        const Eigen::Map<const Eigen::ArrayXd> fitY(y.data() + startIdx, size);

        const auto lrResult = Stats::linearRegression(fitX, fitY, true);
        const double slope = lrResult.slope;
        const double intercept = Utils::positiveFmod(lrResult.intercept, interceptMod);

        if (!std::isfinite(slope) || std::abs(slope) > maxSlope) {
            return;
        }

        slopeWeights.append(slope, intercept, *lrResult.mse, size);
        LOG_DEBUG("Using slope ", slope, " and intercept ", intercept, " for block [", startIdx, ", ", endIdx, ") with size ", size);
    }

    void SegmentedMedianSlopeEstimator::processBlockResolution(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const int32_t startIdx, const int32_t endIdx, Eigen::ArrayXd &out,
        int32_t& writeCount
    ) const {
            const int size = endIdx - startIdx;
            if (size <= 2) {
                return;
            }

            const Eigen::Map<const Eigen::ArrayXd> fitX(x.data() + startIdx, size);
            const Eigen::Map<const Eigen::ArrayXd> fitY(y.data() + startIdx, size);

            out.segment(writeCount, size) = Utils::diff(fitY) / Utils::diff(fitX);
            writeCount += size;
    }

    void SegmentedMedianSlopeEstimator::SlopesWeights::reserve(const uint64_t count) {
        slopes.reserve(count);
        weights.reserve(count);
    }

    void SegmentedMedianSlopeEstimator::SlopesWeights::append(const double slope, const double intercept, const double mse, const int32_t weight) {
        slopes.emplace_back(slope);
        intercepts.emplace_back(intercept);
        rmses.emplace_back(std::sqrt(mse));
        weights.emplace_back(weight);
    }

    uint64_t SegmentedMedianSlopeEstimator::SlopesWeights::count() const {
        return slopes.size();
    }
} // accurate_ri
