#pragma once
#include <cstdint>
#include <eigen3/Eigen/Core>

namespace accurate_ri {
    class MadResolutionLoss {
        double computeResolutionLoss(
            const Eigen::ArrayX<double> &invRangesXy, const Eigen::ArrayX<double> &thetas, uint32_t resolution
        );
    };
}
