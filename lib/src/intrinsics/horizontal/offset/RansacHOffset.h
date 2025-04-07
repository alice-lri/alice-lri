#pragma once
#include <optional>
#include "point/PointArray.h"

namespace accurate_ri {
    struct RansacHOffsetResult {
        double offset;
        double loss;
    };

    class RansacHOffset {
    public:
        static std::optional<RansacHOffsetResult> computeOffset(
            const Eigen::ArrayXd &invRangesXy, const Eigen::ArrayXd &thetas, const uint32_t resolution,
            const double coordsEps
        );
    };
} // accurate_ri
