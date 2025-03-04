#pragma once
#include <cstdint>
#include <eigen3/Eigen/Core>

namespace accurate_ri {

class CustomRansac {
private:
    const uint32_t minSamples;
    const double residualThreshold;
    const uint32_t maxTrials;
    const uint32_t resolution;

public:
    CustomRansac(
        const uint32_t minSamples, const double residualThreshold, const uint32_t maxTrials, const uint32_t resolution
    ) : minSamples(minSamples),
        residualThreshold(residualThreshold),
        maxTrials(maxTrials),
        resolution(resolution) {}

    void fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
};

} // accurate_ri
