#include "SegmentedMedianLinearRegressor.h"

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "math/Stats.h"
#include "utils/logger/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    Stats::LRResult SegmentedMedianLinearRegressor::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const {
        const Segments segments = segmentAndFit(x, y);

        if (segments.count() == 0) {
            return Stats::LRResult(0,0);
        }

        return Stats::LRResult(
            Stats::weightedMedian(segments.slopes, segments.weights),
            Stats::weightedMedian(segments.intercepts, segments.weights),
            std::nullopt
        );
    }

    SegmentedMedianLinearRegressor::Segments SegmentedMedianLinearRegressor::segmentAndFit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y
    ) const {
        Segments segments;
        const int32_t n = static_cast<int32_t>(x.size());
        const Eigen::ArrayX<bool> continuityMask =
                (Utils::diff(x).abs() < segmentThresholdX) && (Utils::diff(y).abs() < segmentThresholdY);

        int blockStart = 0;
        for (int i = 1; i < n; ++i) {
            if (!continuityMask[i - 1]) {
                processSegment(x, y, blockStart + 1, i, segments);
                blockStart = i;
            }
        }

        processSegment(x, y, blockStart + 1, n, segments);
        return segments;
    }

    void SegmentedMedianLinearRegressor::processSegment(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const int32_t startIdx, const int32_t endIdx,
        Segments& slopeWeights
    ) const {
        const int size = endIdx - startIdx;
        if (size <= 2) {
            return;
        }

        const Eigen::Map<const Eigen::ArrayXd> fitX(x.data() + startIdx, size);
        const Eigen::Map<const Eigen::ArrayXd> fitY(y.data() + startIdx, size);

        const auto lrResult = Stats::linearRegression(fitX, fitY, false);
        const double slope = lrResult.slope;
        const double intercept = Utils::positiveFmod(lrResult.intercept, interceptMod);

        if (!std::isfinite(slope) || std::abs(slope) > maxSlope) {
            return;
        }

        slopeWeights.append(slope, intercept, size);
        LOG_DEBUG("Using slope ", slope, " and intercept ", intercept, " for block [", startIdx, ", ", endIdx, ") with size ", size);
    }

    void SegmentedMedianLinearRegressor::Segments::reserve(const uint64_t count) {
        slopes.reserve(count);
        weights.reserve(count);
    }

    void SegmentedMedianLinearRegressor::Segments::append(
        const double slope, const double intercept,const int32_t weight
    ) {
        slopes.emplace_back(slope);
        intercepts.emplace_back(intercept);
        weights.emplace_back(weight);
    }

    uint64_t SegmentedMedianLinearRegressor::Segments::count() const {
        return slopes.size();
    }
} // accurate_ri
