#pragma once
#include <cstdint>
#include <eigen3/Eigen/Core>

namespace accurate_ri::HorizontalMath {
    Eigen::ArrayXd computeDiffToIdeal(const Eigen::ArrayXd &thetas, uint32_t resolution, bool reconstruct);
} // accurate_ri
