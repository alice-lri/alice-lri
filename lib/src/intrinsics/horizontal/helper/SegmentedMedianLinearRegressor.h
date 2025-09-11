# pragma once
#include <vector>

#include "math/LinearRegressor.h"
#include "math/Stats.h"

namespace accurate_ri {
    class SegmentedMedianLinearRegressor {
    private:
        struct Segments {
            std::vector<double> slopes;
            std::vector<double> intercepts;
            std::vector<int32_t> weights;

            void reserve(uint64_t count);

            void append(double slope, double intercept, int32_t weight);

            [[nodiscard]] uint64_t count() const;
        };

        const double segmentThresholdX;
        const double segmentThresholdY;
        const double maxSlope;
        const double interceptMod;

    public:
        SegmentedMedianLinearRegressor(
            const double segmentThresholdX, const double segmentThresholdY, const double maxSlope,
            const double interceptMod
        ) : segmentThresholdX(segmentThresholdX),
            segmentThresholdY(segmentThresholdY),
            maxSlope(maxSlope),
            interceptMod(interceptMod) {}

        [[nodiscard]] LRResult fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;

    private:
        void processSegment(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, int32_t startIdx, int32_t endIdx,
            Segments &slopeWeights
        ) const;

        [[nodiscard]] Segments segmentAndFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;
    };
} // accurate_ri
