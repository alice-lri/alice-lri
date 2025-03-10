#include "RansacHOffset.h"

#include <accurate_ri/accurate_ri.hpp>
#include <cstdint>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "point/PointArray.h"
#include "ransac/CustomRansac.h"
#include "utils/Logger.h"

namespace accurate_ri {
    std::optional<double> RansacHOffset::computeOffset(
        const Eigen::ArrayXd &invRangesXy, const Eigen::ArrayXd &thetas, const uint32_t resolution,
        const double coordsEps
    ) {
        auto diffToIdeal = HorizontalMath::computeDiffToIdeal(thetas, resolution, false);
        double residualThreshold = coordsEps * 1.5;
        double thetaStep = 2 * M_PI / resolution;

        if (getResidualThreshold()) {
            residualThreshold = *getResidualThreshold();
        }

        CustomRansac ransac = CustomRansac(2, residualThreshold, 1000, thetaStep);
        const std::optional<Stats::LRResult> ransacResult = ransac.fit(invRangesXy, diffToIdeal);

        return ransacResult.has_value() ? std::optional(ransacResult->slope) : std::nullopt;
    }
} // accurate_ri
