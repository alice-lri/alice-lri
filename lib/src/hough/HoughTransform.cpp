#include "HoughTransform.h"

#include <algorithm>
#include <cmath>

#include "hash/HashUtils.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "utils/Logger.h"

namespace accurate_ri {

    HoughTransform::HoughTransform(
        const double xMin, const double xMax, const double xStep, const double yMin, const double yMax,
        const double yStep
    ) : xMin(xMin), xMax(xMax), xStep(xStep), yMin(yMin), yMax(yMax), yStep(yStep) {
        xCount = std::floor((xMax - xMin + xStep) / xStep);
        yCount = std::floor((yMax - yMin + yStep) / yStep);

        accumulator = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(yCount, xCount);
        hashAccumulator = Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(yCount, xCount);

        LOG_DEBUG("HoughTransform initialized with xCount: ", xCount, " yCount: ", yCount);
    }

    void HoughTransform::computeAccumulator(const PointArray &points) {
        LOG_DEBUG("Starting accumulator computation for ", points.size(), " points");

        for (uint64_t i = 0; i < points.size(); i++) {
            if (i % 10000 == 0) {
                LOG_DEBUG("Processing point ", i, " of ", points.size());
            }
            updateAccumulatorForPoint(i, points, VoteType::RANGE);
        }

        LOG_DEBUG("Accumulator computation completed.");
    }

    inline void HoughTransform::updateAccumulatorForPoint(
        const uint64_t pointIndex, const PointArray &points, const VoteType &voteType
    ) {
        int32_t previousY = -1;

        for (size_t x = 0; x < xCount; x++) {
            const double rangeVal = points.getRange(pointIndex);
            const double yVal = points.getPhi(pointIndex) - asin(getXValue(x) / rangeVal);
            const auto y = static_cast<int32_t>(std::round((yVal - yMin) / yStep));

            if (y < 0 || y >= yCount) {
                continue;
            }

            const double voteVal = voteType == VoteType::RANGE ? rangeVal : 0;

            if (previousY != -1) {
                // TODO this is inside the if for consistency with Python, but maybe is not the right thing to do
                accumulator(y, x) = (voteType != VoteType::ZERO) ? (accumulator(y, x) + voteVal) : 0;
                hashAccumulator(y, x) ^= HashUtils::knuth_uint(pointIndex);

                voteForDiscontinuities(pointIndex, x, y, voteVal, previousY);
            }

            previousY = y;
        }
    }

    inline void HoughTransform::voteForDiscontinuities(
        const uint64_t pointIndex, const size_t x, const int32_t y, const double voteVal, const int32_t previousY
    ) {
        assert(x > 0);

        const int32_t yMin = std::min(previousY, y);
        const int32_t yMax = std::max(previousY, y);

        if (yMin == yMax) {
            return;
        }

        accumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).array() += voteVal;

        hashAccumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).array() =
                hashAccumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).unaryExpr(
                    [&](uint64_t val) {
                        return val ^ HashUtils::knuth_uint(pointIndex); // Equivalent to ^= but for Eigen
                    }
                );
    }

    std::optional<HoughCell> HoughTransform::findMaximum(std::optional<double> averageX) {
        std::vector<std::pair<size_t, size_t> > maxIndices;
        double maxVal = -std::numeric_limits<double>::infinity();

        for (size_t y = 0; y < yCount; y++) {
            for (size_t x = 0; x < xCount; x++) {
                const double val = accumulator(y * xCount + x);
                if (val > maxVal) {
                    maxVal = val;
                    maxIndices = {{x, y}};
                } else if (val == maxVal) {
                    maxIndices.emplace_back(x, y);
                }
            }
        }

        if (maxVal <= 1e-6 || maxIndices.empty()) {
            LOG_INFO("No maxima found in the accumulator.");
            return std::nullopt;
        }

        if (maxIndices.size() == 1 || !averageX) {
            return indicesToCell(maxIndices[maxIndices.size() / 2]);
        }

        auto closestPair = maxIndices[0];
        double minDistance = std::numeric_limits<double>::infinity();

        for (const auto &[x, y]: maxIndices) {
            const double distance = std::abs(getXValue(x) - *averageX);

            if (distance < minDistance) {
                minDistance = distance;
                closestPair = {x, y};
            }
        }


        return indicesToCell(closestPair);
    }

    void HoughTransform::eraseByHash(const uint64_t hash) {
        accumulator = (hashAccumulator.array() == hash).select(0, accumulator.array());
    }

    void HoughTransform::restoreVotes(const uint64_t hash, const double votes) {
        accumulator = (hashAccumulator.array() == hash).select(votes, accumulator.array());
    }

    void HoughTransform::eraseWhere(const PointArray &points, const Eigen::ArrayXi &indices) {
        for (const int32_t index : indices) {
            updateAccumulatorForPoint(index, points, VoteType::ZERO);
        }
    }

    // TODO this is a debug function, should be removed
    void HoughTransform::ensureHashEquals(Eigen::Matrix<uint64_t, -1, -1> &matrix) {
        assert(matrix.rows() == hashAccumulator.rows());
        assert(matrix.cols() == hashAccumulator.cols());

        bool diffFlag = false;
        for (int i = 0; i < matrix.rows(); i++) {
            for (int j = 0; j < matrix.cols(); j++) {
                if (matrix(i, j) != hashAccumulator(i, j)) {
                    diffFlag = true;
                    LOG_DEBUG("Hashes do not match at ", i, ", ", j);
                    LOG_DEBUG("Expected: ", matrix(i, j), ", got: ", hashAccumulator(i, j));
                }
            }
        }

        if (diffFlag) {
            LOG_DEBUG("Hashes do not match");
        } else {
            LOG_DEBUG("Hashes match");
        }
    }

    // TODO this is also a debug function, should be removed
    void HoughTransform::ensureAccEquals(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &matrix) {
        assert(matrix.rows() == accumulator.rows());
        assert(matrix.cols() == accumulator.cols());

        bool diffFlag = false;
        for (int i = 0; i < matrix.rows(); i++) {
            for (int j = 0; j < matrix.cols(); j++) {
                if (matrix(i, j) != accumulator(i, j)) {
                    diffFlag = true;
                    LOG_DEBUG("Accumulators do not match at ", i, ", ", j);
                    LOG_DEBUG("Expected: ", matrix(i, j), ", got: ", accumulator(i, j));
                }
            }
        }

        if (diffFlag) {
            LOG_DEBUG("Accumulators do not match");
        } else {
            LOG_DEBUG("Accumulators match");
        }
    }

    HoughCell HoughTransform::indicesToCell(const std::pair<size_t, size_t> &indices) {
        return {
            indices.first,
            indices.second,
            getXValue(indices.first),
            getYValue(indices.second),
            accumulator(indices.second, indices.first),
            hashAccumulator(indices.second, indices.first)
        };
    }

    double HoughTransform::getXValue(const size_t index) const {
        return xMin + xStep * static_cast<double>(index);
    }

    double HoughTransform::getYValue(const size_t index) const {
        return yMin + yStep * static_cast<double>(index);
    }
} // namespace accurate_ri
