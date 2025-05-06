#include "HorizontalMath.h"
#include <cstdint>
#include <numeric>
#include <numbers>
#include "utils/Utils.h"

namespace accurate_ri::HorizontalMath {
    Eigen::ArrayXd computeDiffToIdeal(const Eigen::ArrayXd &thetas, const uint32_t resolution, const bool reconstruct) {
        const double thetaStep = 2 * std::numbers::pi / static_cast<double>(resolution);
        const auto closestIdeal = (thetas / thetaStep).round() * thetaStep;
        Eigen::ArrayXd diffToIdeal = thetas - closestIdeal;

        if (reconstruct) {
            const auto diffDiffToIdeal = Utils::diff(diffToIdeal);
            const auto jumpMask = (diffDiffToIdeal.abs() >= thetaStep / 1.5).cast<double>();
            const auto signs = diffDiffToIdeal.sign().cast<double>();
            const Eigen::ArrayXd diffDiffToIdealNoJumps = diffDiffToIdeal - thetaStep * jumpMask * signs;

            // Diff to ideal is the cumulative sum of diffDiffToIdealNoJumps, with the first element being 0
            diffToIdeal[0] = 0;
            std::partial_sum(diffDiffToIdealNoJumps.begin(), diffDiffToIdealNoJumps.end(), diffToIdeal.begin() + 1);
        }

        return diffToIdeal;
    }
} // accurate_ri
