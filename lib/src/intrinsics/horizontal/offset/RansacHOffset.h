#pragma once
#include <optional>

#include "point/PointArray.h"

namespace accurate_ri {
    class RansacHOffset {
    public:
        static std::optional<double> computeOffset(
            const Eigen::ArrayXd &invRangesXy, const Eigen::ArrayXd &thetas, const uint32_t resolution,
            const double coordsEps
        );
    };
} // accurate_ri
