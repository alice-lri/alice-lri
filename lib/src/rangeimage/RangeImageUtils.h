#pragma once
#include "accurate_ri/public_structs.hpp"
#include <Eigen/Core>

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    );

    PointCloud::Double unProjectRangeImage(const IntrinsicsResult &intrinsics, const RangeImage &image);
}
