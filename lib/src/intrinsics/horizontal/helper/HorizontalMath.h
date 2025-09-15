#pragma once
#include <Eigen/Core>

namespace alice_lri::HorizontalMath {
    Eigen::ArrayXd computeDiffToIdeal(const Eigen::ArrayXd &thetas, uint32_t resolution, bool reconstruct);
}
