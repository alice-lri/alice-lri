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

    std::optional<std::pair<uint64_t, uint64_t>> HoughTransform::findMaximum(std::optional<double> averageX) {
        std::vector<std::pair<size_t, size_t>> maxIndices;
        double maxVal = -std::numeric_limits<double>::infinity();

        for (size_t y = 0; y < yCount; y++) {
            for (size_t x = 0; x < xCount; x++) {
                const double val = accumulator[y * xCount + x];
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

        for (const auto &[x, y] : maxIndices) {
            const double xValue = xMin + xStep * static_cast<double>(x);
            const double distance = std::abs(xValue - *averageX);

            if (distance < minDistance) {
                minDistance = distance;
                closestPair = {x, y};
            }
        }

        return closestPair;
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
