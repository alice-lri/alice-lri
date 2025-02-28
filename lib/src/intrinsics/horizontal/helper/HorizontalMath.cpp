#include "HorizontalMath.h"
#include <cstdint>
#include <cmath>
#include <numbers>
#include <eigen3/Eigen/src/Core/Array.h>
#include <eigen3/Eigen/src/Core/Array.h>

#include "utils/Utils.h"


namespace accurate_ri::HorizontalMath {
    Eigen::ArrayXd computeDiffToIdeal(const Eigen::ArrayXd& thetas, uint32_t resolution, bool reconstruct) {
        const double thetaStep = 2 * std::numbers::pi / static_cast<double>(resolution);
        const auto closestIdeal = (thetas / thetaStep).round() * resolution;
        Eigen::ArrayXd diffToIdeal = thetas - closestIdeal;

        // TODO i think this can be done in a single pass
        if (reconstruct) {
            const auto diffDiffToIdeal = Utils::diff(diffToIdeal);
            const auto jumpMask = diffDiffToIdeal.cabs() >= thetaStep / 1.5;

            diffToIdeal[0] = 0;
            for (int i = 0; i < diffDiffToIdeal.size(); ++i) {
                if (jumpMask[i]) {
                    diffDiffToIdeal[i] -= thetaStep * Utils::sign(diffDiffToIdeal[i]);
                }
                diffToIdeal[i + 1] = diffToIdeal[i] + diffDiffToIdeal[i];
            }
        }
    }
} // accurate_ri
