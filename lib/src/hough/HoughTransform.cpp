#include "HoughTransform.h"

#include <algorithm>
#include <BuildOptions.h>
#include <cmath>

#include "hash/HashUtils.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

namespace accurate_ri {
    HoughTransform::HoughTransform(
        const double xMin, const double xMax, const double xStep, const double yMin, const double yMax,
        const double yStep
    ) : xMin(xMin), xMax(xMax), xStep(xStep), yMin(yMin), yMax(yMax), yStep(yStep) {
        xCount = std::floor((xMax - xMin + xStep) / xStep);
        yCount = std::floor((yMax - yMin + yStep) / yStep);

        accumulator = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(yCount, xCount);
        hashAccumulator = Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(yCount, xCount);

        LOG_DEBUG("HoughTransform initialized with xCount: ", xCount, " yCount: ", yCount);
    }

    void HoughTransform::computeAccumulator(const PointArray &points) {
        PROFILE_SCOPE("HoughTransform::computeAccumulator");
        LOG_DEBUG("Starting accumulator computation for ", points.size(), " points");

        for (uint64_t i = 0; i < points.size(); i++) {
            if (i % 10000 == 0) {
                LOG_DEBUG("Processing point ", i, " of ", points.size());
            }
            updateAccumulatorForPoint(i, points, HoughOperation::ADD, HoughMode::VOTES_AND_HASHES);
        }

        LOG_DEBUG("Accumulator computation completed.");
    }

    inline void HoughTransform::updateAccumulatorForPoint(
        const uint64_t pointIndex, const PointArray &points, const HoughOperation operation, const HoughMode mode
    ) {
        int32_t previousY = -1;

        for (int64_t x = 0; x < xCount; x++) {
            const double rangeVal = points.getRange(pointIndex);
            const double yVal = points.getPhi(pointIndex) - getXValue(x) / rangeVal;
            const auto y = static_cast<int32_t>(std::round((yVal - yMin) / yStep));

            if (y < 0 || y >= yCount) {
                continue;
            }

            if (previousY != -1) {
                accumulator(y, x) += operation == HoughOperation::ADD? 1 : -1;
                if (mode == HoughMode::VOTES_AND_HASHES) {
                    hashAccumulator(y, x) ^= HashUtils::knuthHash(pointIndex);
                }

                if constexpr (BuildOptions::USE_HOUGH_CONTINUITY) {
                    voteForDiscontinuities(pointIndex, x, y, previousY, operation, mode);
                }
            }

            previousY = y;
        }
    }

    inline void HoughTransform::voteForDiscontinuities(
        const uint64_t pointIndex, const int64_t x, const int32_t y, const int32_t previousY,
        const HoughOperation operation, const HoughMode mode
    ) {
        assert(x > 0);

        const int32_t minY = std::min(previousY, y);
        const int32_t maxY = std::max(previousY, y);

        if (minY == maxY) {
            return;
        }

        auto &&accumulatorBlock = accumulator.block(minY + 1, x - 1, maxY - minY - 1, 2).array();
        accumulatorBlock += operation == HoughOperation::ADD? 1 : -1;

        if (mode == HoughMode::VOTES_AND_HASHES) {
            hashAccumulator.block(minY + 1, x - 1, maxY - minY - 1, 2).array() =
                hashAccumulator.block(minY + 1, x - 1, maxY - minY - 1, 2).unaryExpr(
                    [&](const uint64_t val) {
                        return val ^ HashUtils::knuthHash(pointIndex); // Equivalent to ^= but for Eigen
                    }
                );
        }
    }

    std::optional<HoughCell> HoughTransform::findMaximum(const std::optional<double> averageX) {
        PROFILE_SCOPE("HoughTransform::findMaximum");
        std::vector<std::pair<size_t, size_t> > maxIndices;
        int64_t maxVotes = 0;

        for (int64_t y = 0; y < yCount; y++) {
            for (int64_t x = 0; x < xCount; x++) {
                const int64_t votes = accumulator(y, x);
                if (votes > maxVotes && votes > 0) {
                    maxVotes = votes;
                    maxIndices = {{x, y}};
                } else if (votes == maxVotes) {
                    maxIndices.emplace_back(x, y);
                }
            }
        }

        if (maxVotes <= 0 || maxIndices.empty()) {
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
        PROFILE_SCOPE("HoughTransform::eraseByHash");
        accumulator = (hashAccumulator.array() == hash).select(0, accumulator.array());
    }

    void HoughTransform::restoreVotes(const uint64_t hash, const int64_t votes) {
        accumulator = (hashAccumulator.array() == hash).select(votes, accumulator.array());
    }

    void HoughTransform::addVotes(const PointArray &points, const Eigen::ArrayXi &indices) {
        PROFILE_SCOPE("HoughTransform::addVotes");
        for (const int32_t index: indices) {
            updateAccumulatorForPoint(index, points, HoughOperation::ADD, HoughMode::VOTES_ONLY);
        }
    }

    void HoughTransform::removeVotes(const PointArray &points, const Eigen::ArrayXi &indices) {
        PROFILE_SCOPE("HoughTransform::removeVotes");
        for (const int32_t index: indices) {
            updateAccumulatorForPoint(index, points, HoughOperation::SUBTRACT, HoughMode::VOTES_ONLY);
        }
    }

    HoughCell HoughTransform::indicesToCell(const std::pair<int64_t, int64_t> &indices) {
        return {
            static_cast<uint64_t>(indices.first),
            static_cast<uint64_t>(indices.second),
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
