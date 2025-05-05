#include "MadResolutionLoss.h"
#include <cstdint>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    template<typename T>
    Eigen::ArrayXd computeDiffThetaIdealWrtDiffRxy(
        const Eigen::ArrayBase<T> &invRangesXy, const Eigen::ArrayBase<T> &thetas, uint32_t resolution
    );

    double MadResolutionLoss::computeResolutionLoss(
        const Eigen::ArrayXd &invRangesXy, const Eigen::ArrayXd &thetas, const uint32_t resolution
    ) {
        auto df = computeDiffThetaIdealWrtDiffRxy(invRangesXy, thetas, resolution);
        const double median = Utils::medianInPlace(df);
        const double mad = (df - median).abs().mean() * resolution;

        if (resolution == 4000) {
            LOG_INFO("MAD median (offset): ", median);
        }

        return mad;
    }

    template<typename T>
    Eigen::ArrayXd computeDiffThetaIdealWrtDiffRxy(
        const Eigen::ArrayBase<T> &invRangesXy, const Eigen::ArrayBase<T> &thetas, const uint32_t resolution
    ) {
        const auto diffToIdeal = HorizontalMath::computeDiffToIdeal(thetas, resolution, false);
        const auto diffInvRangesXy = Utils::diff(invRangesXy);
        const auto diffInvRangesXyEpsMask = diffInvRangesXy.abs() >= 1e-7; // TODO optimize this is the same every time

        if (diffInvRangesXyEpsMask.count() < 2) {
            return Utils::diff(diffToIdeal) / diffInvRangesXy;
        }

        std::vector<int32_t> indicesVector;
        indicesVector.reserve(diffInvRangesXyEpsMask.size());
        for (int i = 0; i < diffInvRangesXyEpsMask.size(); ++i) {
            if (diffInvRangesXyEpsMask[i]) {
                indicesVector.emplace_back(i);
            }
        }

        const auto indices = Eigen::Map<const Eigen::ArrayXi>(indicesVector.data(), indicesVector.size());

        return Utils::diff(diffToIdeal)(indices) / diffInvRangesXy(indices);
    }
}
