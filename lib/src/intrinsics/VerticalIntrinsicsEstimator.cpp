#include "VerticalIntrinsicsEstimator.h"

#include <algorithm>

namespace accurate_ri {
    constexpr uint64_t MAX_ITERATIONS = 10000;

    void VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        initHough(points);

        hough->computeAccumulator(points);

        int64_t unassignedPoints = points.size();
        int64_t iteration = -1;

        while (unassignedPoints > 0) {
            iteration++;
            if (iteration > MAX_ITERATIONS) {
                break;
            }

            const std::optional<std::pair<uint64_t, uint64_t>> maxIndices = hough->findMaximum(std::nullopt);

            if (!maxIndices) {
                break;
            }

            const auto [maxOffset, maxAngle] = *maxIndices;



        }
    }

    // TODO extract these constants somewhere
    void VerticalIntrinsicsEstimator::initHough(const PointArray &points) {
        double offsetStep = 1e-3;
        double offsetMax = std::min(std::ranges::min(points.getRanges()), 0.5) - offsetStep;
        double offsetMin = -offsetMax;

        double angleStep = 1e-4;
        double angleMax = M_PI / 2 - angleStep;
        double angleMin = -angleMax;

        hough = std::make_unique<HoughTransform>(offsetMin, offsetMax, offsetStep, angleMin, angleMax, angleStep);
    }
} // namespace accurate_ri
