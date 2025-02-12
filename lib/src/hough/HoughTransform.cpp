#include "HoughTransform.h"

#include <cmath>

#include "hash/HashUtils.h"
#include "utils/Logger.h"

namespace accurate_ri {
    HoughTransform::HoughTransform(
        const double xMin, const double xMax, const double xStep, const double yMin, const double yMax,
        const double yStep
    ) : xMin(xMin), xMax(xMax), xStep(xStep), yMin(yMin), yMax(yMax), yStep(yStep) {
        xCount = std::floor((xMax - xMin) / xStep);
        yCount = std::floor((yMax - yMin) / yStep);

        accumulator.resize(xCount * yCount);
        hashAccumulator.resize(xCount * yCount);

        LOG_INFO("HoughTransform initialized with xCount:", xCount, "yCount:", yCount);
    }

    void HoughTransform::computeAccumulator(const PointArray &points) {
        auto xValues = std::vector<double>(xCount);

        for (size_t i = 0; i < xCount; i++) {
            xValues[i] = xMin + xStep * static_cast<double>(i);
        }

        LOG_INFO("Starting accumulator computation for", points.size(), "points");

        for (uint64_t i = 0; i < points.size(); i++) {
            updateAccumulatorForPoint(i, points, xValues);
        }

        LOG_INFO("Accumulator computation completed.");
    }

    inline void HoughTransform::updateAccumulatorForPoint(
        const uint64_t pointIndex, const PointArray &points, const std::vector<double> &xValues
    ) {
        int32_t previousY = -1;

        for (size_t x = 0; x < xCount; x++) {
            const double xVal = xValues[x];
            const double rangeVal = points.getRange(pointIndex);
            const double yVal = points.getPhi(pointIndex) - asin(xVal / rangeVal);
            const auto y = static_cast<int32_t>(std::round((yVal - yMin) / yStep));

            if (y < 0 || y >= yCount) {
                continue;
            }

            const double voteVal = rangeVal;

            accumulator[y * xCount + x] += voteVal;
            hashAccumulator[y * xCount + x] ^= HashUtils::knuth_uint(pointIndex);

            if (previousY != -1) {
                voteForDiscontinuities(pointIndex, x, y, voteVal, previousY);
            }

            previousY = y;
        }
    }

    inline void HoughTransform::voteForDiscontinuities(
        const uint64_t pointIndex, const size_t x, const int32_t y, const double voteVal, const int32_t previousY
    ) {
        const int32_t yMin = std::min(previousY, y);
        const int32_t yMax = std::max(previousY, y);

        // We do not vote only the current y cell, but also the cells between the
        // previous and current y This is done to avoid discontinuities in the lines
        // of the accumulator
        for (int32_t yBetween = yMin + 1; yBetween < yMax; yBetween++) {
            accumulator[yBetween * xCount + x - 1] += voteVal;
            accumulator[yBetween * xCount + x] += voteVal;

            hashAccumulator[yBetween * xCount + x - 1] ^= HashUtils::knuth_uint(pointIndex);
            hashAccumulator[yBetween * xCount + x] ^= HashUtils::knuth_uint(pointIndex);
        }
    }
} // namespace accurate_ri
