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

            void reserveBlocks(uint64_t count);
            void appendNewBlock(uint64_t reservePerBlock);
            void appendToLastBlock(double x, double y);

            [[nodiscard]] double computeSlope(int32_t blockIdx) const;

            [[nodiscard]] uint64_t count() const;
        };

        struct SlopesWeights {
            std::vector<double> slopes;
            std::vector<int32_t> weights;

            void reserve(uint64_t count);
            void append(double slope, int32_t weight);
            [[nodiscard]] uint64_t count() const;
        };

        const double segmentThreshold;
        const double maxSlope;

    public:
        SegmentedMedianSlopeEstimator(const double segmentThreshold, const double maxSlope)
            : segmentThreshold(segmentThreshold), maxSlope(maxSlope) {}

        [[nodiscard]] SlopesWeights computeBlocksSlopesWeights(const Blocks &blocks) const;

        [[nodiscard]] double estimateSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;

    private:
        [[nodiscard]] Blocks segmentIntoBlocks(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const;
    };
} // accurate_ri
