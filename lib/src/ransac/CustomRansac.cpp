#include "CustomRansac.h"

#include <limits>
#include <eigen3/Eigen/src/Core/Array.h>

#include "utils/Logger.h"

namespace accurate_ri {
    void CustomRansac::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        uint32_t trial = 0;

        for (trial = 0; trial < maxTrials; ++trial) {
            const uint32_t sampleIndex1 = std::rand() % y.size();
            uint32_t sampleIndex2 = std::rand() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size(); // Bias is fine

            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            const Stats::LRResult& currentModel = estimator.fit(sampleX, sampleY);
            const auto residuals = estimator.residuals(x, y);
            const uint32_t inliersCount = (residuals < residualThreshold).count();

            if (inliersCount == x.size()) {
                model = currentModel;
                break;
            }
        }

        if (trial == maxTrials) {
            LOG_ERROR("RANSAC could not find a valid consensus set at trial ", trial);
            return;
        }
    }

    void CustomRansac::refineSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {

    }

} // accurate_ri
