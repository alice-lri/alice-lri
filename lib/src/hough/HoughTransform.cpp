#include "HoughTransform.h"

#include <algorithm>
#include <cmath>

#include "hash/HashUtils.h"
#include "utils/Logger.h"

namespace accurate_ri {
    HoughTransform::HoughTransform(
        const double xMin, const double xMax, const double xStep, const double yMin, const double yMax,
        const double yStep
    ) : xMin(xMin), xMax(xMax), xStep(xStep), yMin(yMin), yMax(yMax), yStep(yStep) {
        accumulator = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
            static_cast<int>(std::floor((yMax - yMin) / yStep)),
            static_cast<int>(std::floor((xMax - xMin) / xStep))
        );

        hashAccumulator = Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
            static_cast<int>(std::floor((yMax - yMin) / yStep)),
            static_cast<int>(std::floor((xMax - xMin) / xStep))
        );

        xCount = std::floor((xMax - xMin) / xStep);
        yCount = std::floor((yMax - yMin) / yStep);

        LOG_INFO("HoughTransform initialized with xCount:", xCount, "yCount:", yCount);
    }

    void HoughTransform::computeAccumulator(const PointArray &points) {
        LOG_INFO("Starting accumulator computation for", points.size(), "points");

        for (uint64_t i = 0; i < points.size(); i++) {
            updateAccumulatorForPoint(i, points);
        }

        LOG_INFO("Accumulator computation completed.");
    }

    inline void HoughTransform::updateAccumulatorForPoint(const uint64_t pointIndex, const PointArray &points) {
        int32_t previousY = -1;

        for (size_t x = 0; x < xCount; x++) {
            const double rangeVal = points.getRange(pointIndex);
            const double yVal = points.getPhi(pointIndex) - asin(getXValue(x) / rangeVal);
            const auto y = static_cast<int32_t>(std::round((yVal - yMin) / yStep));

            if (y < 0 || y >= yCount) {
                continue;
            }

            const double voteVal = rangeVal;

            accumulator(y, x) += voteVal;
            hashAccumulator(y, x) ^= HashUtils::knuth_uint(pointIndex);

            if (previousY != -1) {
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

        accumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).array() += voteVal;

        hashAccumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).array() =
                hashAccumulator.block(yMin + 1, x - 1, yMax - yMin - 1, 2).unaryExpr(
                    [&](uint64_t val) {
                        return val ^ HashUtils::knuth_uint(pointIndex); // Equivalent to ^= but for Eigen
                    }
                );
    }

    std::optional<std::pair<uint64_t, uint64_t> > HoughTransform::findMaximum(std::optional<double> averageX) {
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
            return maxIndices[maxIndices.size() / 2];
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

        return closestPair;
    }

    double HoughTransform::getXValue(const size_t index) const {
        return xMin + xStep * static_cast<double>(index);
    }

    double HoughTransform::getYValue(const size_t index) const {
        return yMin + yStep * static_cast<double>(index);
    }
} // namespace accurate_ri
