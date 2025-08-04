# pragma once
#include <vector>
#include <Eigen/Dense>

#include "math/Stats.h"

namespace accurate_ri {
    class SegmentedMedianSlopeEstimator {
    private:
        struct SlopesWeights {
            std::vector<double> slopes;
            std::vector<double> intercepts;
            std::vector<double> rmses;
            std::vector<int32_t> weights;

            void reserve(uint64_t count);

            void append(double slope, double intercept, double mse, int32_t weight);

            [[nodiscard]] uint64_t count() const;
        };

        const double segmentThresholdX;
        const double segmentThresholdY;
        const double maxSlope;
        const double interceptMod;

    public:
        SegmentedMedianSlopeEstimator(
            const double segmentThresholdX, const double segmentThresholdY, const double maxSlope,
            const double interceptMod
        ) : segmentThresholdX(segmentThresholdX),
            segmentThresholdY(segmentThresholdY),
            maxSlope(maxSlope),
            interceptMod(interceptMod) {}

        [[nodiscard]] Stats::LRResult estimateSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;

        [[nodiscard]] double computeResolutionLoss(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &thetas, uint32_t resolution
        ) const;

    private:
        void processBlock(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, int32_t startIdx, int32_t endIdx,
            SlopesWeights &slopeWeights
        ) const;

        void processBlockResolution(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, int32_t startIdx, int32_t endIdx, Eigen::ArrayXd &out,
            int32_t &writeCount
        ) const;
    };
} // accurate_ri
