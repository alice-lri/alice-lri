#include "MadResolutionLoss.h"

#include <cstdint>
#include <eigen3/Eigen/src/Core/Array.h>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Utils.h"

namespace accurate_ri {
    double MadResolutionLoss::computeResolutionLoss(
        const Eigen::ArrayXd &invRangesXy, const Eigen::ArrayXd &thetas, uint32_t resolution
    ) {
        const auto diffToIdeal = HorizontalMath::computeDiffToIdeal(thetas, resolution, true);
        const auto diffInvRangesXy = Utils::diff(invRangesXy);
        const auto diffInvRangesXyEpsMask = diffInvRangesXy.abs() >= 1e-7;
        const bool ignoreMask = diffInvRangesXyEpsMask.count() < 2;

        if (ignoreMask) {
            Utils::diff(diffToIdeal) / diffInvRangesXy;
        } else {
            std::vector<uint32_t> indices;
            for (int i = 0; i < diffInvRangesXyEpsMask.size(); ++i) {

            }
        }

        return 0;
    }
}
