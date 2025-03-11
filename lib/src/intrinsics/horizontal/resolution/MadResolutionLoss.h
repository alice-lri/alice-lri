#pragma once
#include <cstdint>
#include <Eigen/Core>

namespace accurate_ri {
    class MadResolutionLoss {
    public:
        static double computeResolutionLoss(
            const Eigen::ArrayX<double> &invRangesXy, const Eigen::ArrayX<double> &thetas, uint32_t resolution
        );
    };
}
