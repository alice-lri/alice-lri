#include "SegmentedMedianSlopeEstimator.h"

#include "math/Stats.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    double SegmentedMedianSlopeEstimator::estimateSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) const {
        const Blocks blocks = segmentIntoBlocks(x, y);

        SlopesWeights slopeWeights = computeBlocksSlopesWeights(blocks);

        if (slopeWeights.count() == 0) {
            return 0;
        }

        return Stats::weightedMedian(slopeWeights.slopes, slopeWeights.weights);
    }

    SegmentedMedianSlopeEstimator::Blocks SegmentedMedianSlopeEstimator::segmentIntoBlocks(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y
    ) const {
        Blocks result;
        constexpr int expectedBlocks = 8;
        result.reserveBlocks(expectedBlocks);

        const Eigen::ArrayX<bool> continuityMask = Utils::diff(x).abs() < segmentThreshold;

        for (int i = 0; i < x.size(); ++i) {
            if (i == 0 || !continuityMask[i - 1]) {
                result.appendNewBlock(x.size() / expectedBlocks);
            } else {
                result.appendToLastBlock(x[i], y[i]); // TODO Review this, e.g. first element not added
            }
        }

        return result;
    }

    SegmentedMedianSlopeEstimator::SlopesWeights SegmentedMedianSlopeEstimator::computeBlocksSlopesWeights(
        const Blocks &blocks
    ) const {
        SlopesWeights slopeWeights;

        slopeWeights.reserve(blocks.count());
        for (int i = 0; i < blocks.count(); ++i) {
            if (blocks.blockSizes[i] < 2) {
                continue;
            }

            const double slope = blocks.computeSlope(i);

            if (std::abs(slope) > maxSlope) {
                continue;
            }

            slopeWeights.append(slope, blocks.blockSizes[i]);
            LOG_DEBUG("Using slope ", slope, " for block ", i, " with size ", blocks.blockSizes[i]);
        }

        return slopeWeights;
    }

    void SegmentedMedianSlopeEstimator::Blocks::reserveBlocks(const uint64_t count) {
        xBlocks.reserve(count);
        yBlocks.reserve(count);
        blockSizes.reserve(count);
    }

    void SegmentedMedianSlopeEstimator::Blocks::appendNewBlock(const uint64_t reservePerBlock) {
        yBlocks.emplace_back();
        xBlocks.emplace_back();
        blockSizes.emplace_back();

        if (reservePerBlock > 0) {
            xBlocks.back().reserve(reservePerBlock);
            yBlocks.back().reserve(reservePerBlock);
        }
    }

    void SegmentedMedianSlopeEstimator::Blocks::appendToLastBlock(const double x, const double y) {
        xBlocks.back().emplace_back(x);
        yBlocks.back().emplace_back(y);
        blockSizes.back()++;
    }

    double SegmentedMedianSlopeEstimator::Blocks::computeSlope(const int32_t blockIdx) const {
        const Eigen::ArrayXd fitX = Eigen::Map<const Eigen::ArrayXd>(
            xBlocks[blockIdx].data(), static_cast<Eigen::Index>(xBlocks[blockIdx].size())
        );
        const Eigen::ArrayXd fitY = Eigen::Map<const Eigen::ArrayXd>(
            yBlocks[blockIdx].data(), static_cast<Eigen::Index>(yBlocks[blockIdx].size())
        );

        return Stats::simpleLinearRegression(fitX, fitY).slope;
    }

    uint64_t SegmentedMedianSlopeEstimator::Blocks::count() const {
        return blockSizes.size();
    }

    void SegmentedMedianSlopeEstimator::SlopesWeights::reserve(const uint64_t count) {
        slopes.reserve(count);
        weights.reserve(count);
    }

    void SegmentedMedianSlopeEstimator::SlopesWeights::append(const double slope, const int32_t weight) {
        slopes.emplace_back(slope);
        weights.emplace_back(weight);
    }

    uint64_t SegmentedMedianSlopeEstimator::SlopesWeights::count() const {
        return slopes.size();
    }
} // accurate_ri
