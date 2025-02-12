#include "VerticalIntrinsicsEstimator.h"

namespace accurate_ri {
    void VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        initHough(points);

        hough->computeAccumulator(points);
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
