#include "VerticalIntrinsicsEstimator.h"

#include <algorithm>

namespace accurate_ri {
    // TODO extract these constants somewhere
    constexpr uint64_t MAX_ITERATIONS = 10000;
    constexpr double MAX_OFFSET = 0.5;
    constexpr double OFFSET_STEP = 1e-3;
    constexpr double ANGLE_STEP = 1e-4;

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

    void VerticalIntrinsicsEstimator::initHough(const PointArray &points) {
        double offsetMax = std::min(std::ranges::min(points.getRanges()), MAX_OFFSET) - OFFSET_STEP;
        double offsetMin = -offsetMax;

        double angleMax = M_PI / 2 - ANGLE_STEP;
        double angleMin = -angleMax;

        hough = std::make_unique<HoughTransform>(offsetMin, offsetMax, OFFSET_STEP, angleMin, angleMax, ANGLE_STEP);
    }

    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> computeErrorBounds(
        const PointArray &points, const double maxOffset
    ) {
        double coordsEps = points.getCoordsEps();
        const auto& zs = points.getZ();
        const auto& rangesXy = points.getRangesXy();
        const auto& ranges = points.getRanges();

        const auto rangeXySquared = rangesXy.array().square();
        const auto rangeSquared = ranges.array().square();
        const auto zsOverRangesXy = zs.array() / rangesXy.array();
        const auto sqrtFactor = 1 + zsOverRangesXy.square();

        const auto phisUpperBound = (coordsEps * std::sqrt(2) * zs.cwiseAbs() + coordsEps * rangesXy).array()
                              / (rangeXySquared.array() * sqrtFactor.array());

        const auto correctionUpperBound = maxOffset * coordsEps * std::sqrt(3)
                                    / (rangeSquared.array() * (1 - (maxOffset / ranges.array()).square()).sqrt());

        const auto finalUpperBound = phisUpperBound + correctionUpperBound;


        return {phisUpperBound, correctionUpperBound, finalUpperBound};
    }
} // namespace accurate_ri
