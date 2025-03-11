#pragma once
#include <cstdint>
#include <optional>
#include <Eigen/Core>

#include "math/Stats.h"
#include "ransac/CustomEstimator.h"

namespace accurate_ri {

class CustomRansac {
private:
    const uint32_t minSamples;
    const double residualThreshold;
    const uint32_t maxTrials;
    const double thetaStep;

    CustomEstimator estimator;
    std::optional<Stats::LRResult> model = std::nullopt;

public:
    CustomRansac(
        const uint32_t minSamples, const double residualThreshold, const uint32_t maxTrials, const double thetaStep
    ) : minSamples(minSamples),
        residualThreshold(residualThreshold),
        maxTrials(maxTrials),
        thetaStep(thetaStep),
        estimator(thetaStep){}

    std::optional<Stats::LRResult> fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

private:
    void refineSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
};

} // accurate_ri
