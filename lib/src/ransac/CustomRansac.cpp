#include "CustomRansac.h"
#include <limits>
#include "utils/Logger.h"

namespace accurate_ri {
    struct MultiLineItem {
        double xMean = 0;
        double yMean = 0;
        int64_t count = 0;

        inline void push(const double x, const double y) {
            count++;
            xMean += (x - xMean) / static_cast<double>(count);
            yMean += (y - yMean) / static_cast<double>(count);
        }
    };

    std::optional<Stats::LRResult> CustomRansac::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        uint32_t trial = 0;

        for (trial = 0; trial < maxTrials; ++trial) {
            const uint32_t sampleIndex1 = std::rand() % y.size();
            uint32_t sampleIndex2 = std::rand() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size(); // Bias is fine

            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            const Stats::LRResult &currentModel = estimator.fit(sampleX, sampleY);
            const Eigen::ArrayXd &residuals = estimator.computeResiduals(x, y);
            const uint32_t inliersCount = (residuals < residualThreshold).count();

            if (inliersCount == x.size()) {
                model = currentModel;
                break;
            }
        }

        if (trial == maxTrials) {
            LOG_ERROR("RANSAC could not find a valid consensus set at trial ", trial);
            return std::nullopt;
        }

        refineSlope(x, y);
        return model;
    }

    void CustomRansac::refineSlope(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        const MultiLineResult &multi = estimator.getLastMultiLine();
        std::unordered_map<int64_t, MultiLineItem> multiLineMap;

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            int64_t lineIdx = multi.linesIdx[pointIdx];

            MultiLineItem& multiLineItem = multiLineMap[lineIdx];
            multiLineItem.push(x(pointIdx), y(pointIdx));
        }

        Eigen::ArrayXd shiftedX = Eigen::ArrayXd(x.size());
        Eigen::ArrayXd shiftedY = Eigen::ArrayXd(y.size());

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            const MultiLineItem& multiLineItem = multiLineMap.at(multi.linesIdx[pointIdx]);
            shiftedX(pointIdx) = x(pointIdx) - multiLineItem.xMean;
            shiftedY(pointIdx) = y(pointIdx) - multiLineItem.yMean;
        }

        model->slope = (shiftedX * shiftedY).sum() / shiftedX.square().sum();
    }
} // accurate_ri
