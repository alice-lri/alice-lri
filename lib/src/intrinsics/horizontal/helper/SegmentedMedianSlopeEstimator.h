# pragma once
#include <vector>
#include <Eigen/Dense>

namespace accurate_ri {
    class SegmentedMedianSlopeEstimator {
    private:
        struct Blocks {
            std::vector<std::vector<double>> xBlocks;
            std::vector<std::vector<double>> yBlocks;
            std::vector<int32_t> blockSizes;

            void reserveBlocks(int32_t count);
            void appendNewBlock(int32_t reservePerBlock);
            void appendToLastBlock(double x, double y);

            double computeSlope(int32_t blockIdx) const;

            int32_t count() const;
        };

        struct SlopeWeight {
            double slope;
            int32_t weight;
        };

        struct SlopesWeights {
            std::vector<SlopeWeight> data;
            int32_t totalWeight = 0;

            void reserve(int32_t count);
            void append(double slope, int32_t weight);
            int32_t count() const;
        };

        const double segmentThreshold;
        const double maxSlope;

    public:
        SegmentedMedianSlopeEstimator(const double segmentThreshold, const double maxSlope)
            : segmentThreshold(segmentThreshold), maxSlope(maxSlope) {}

        SlopesWeights computeBlocksSlopesWeights(const SegmentedMedianSlopeEstimator::Blocks &blocks) const;

        static double computeWeightedMedian(SegmentedMedianSlopeEstimator::SlopesWeights &slopeWeights);

        double estimateSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;

    private:
        Blocks segmentIntoBlocks(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;
    };
} // accurate_ri
