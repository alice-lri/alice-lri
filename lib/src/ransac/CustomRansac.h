#pragma once
#include <cstdint>
#include <optional>
#include <eigen3/Eigen/Core>

#include "math/Stats.h"
#include "ransac/CustomEstimator.h"

namespace accurate_ri {

class CustomRansac {
private:
    const uint32_t minSamples;
    const double residualThreshold;
    const uint32_t maxTrials;
    const uint32_t resolution;

    CustomEstimator estimator;
    std::optional<Stats::LRResult> model = std::nullopt;

public:
    CustomRansac(
        const uint32_t minSamples, const double residualThreshold, const uint32_t maxTrials, const uint32_t resolution
    ) : minSamples(minSamples),
        residualThreshold(residualThreshold),
        maxTrials(maxTrials),
        resolution(resolution) {}

    void fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

private:
    void refineSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
};

} // accurate_ri
