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

    // TODO refactor CustomRansac and stop trying to pretend is generic when its not
    std::optional<CustomRansacResult> CustomRansac::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        uint32_t trial = 0;



        for (trial = 0; trial < maxTrials; ++trial) {
            const uint32_t sampleIndex1 = std::rand() % y.size();
            uint32_t sampleIndex2 = std::rand() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size(); // Bias is fine

            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            model = estimator.fit(sampleX, sampleY);
            refineFit(x, y);

            const Eigen::ArrayXd &residuals = estimator.computeResiduals(x, y);
            const uint32_t inliersCount = (residuals < residualThreshold).count();

            if (inliersCount == x.size()) {
                // model = currentModel;
                break;
            }
        }

        if (trial == maxTrials) {
            LOG_DEBUG("RANSAC could not find a valid consensus set at trial ", trial);
            return std::nullopt;
        }

        // const double loss = refineFit(x, y);
        const double loss = 0;
        return std::make_optional<CustomRansacResult>({.model = *model, .loss = loss});
    }

    void CustomRansac::refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        const MultiLineResult &multi = estimator.getLastMultiLine();
        std::unordered_map<int64_t, MultiLineItem> multiLineMap;

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            int64_t lineIdx = multi.linesIdx[pointIdx];

            MultiLineItem& multiLineItem = multiLineMap[lineIdx];
            multiLineItem.push(x(pointIdx), y(pointIdx));
        }

        auto shiftedX = Eigen::ArrayXd(x.size());
        auto shiftedY = Eigen::ArrayXd(y.size());

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            const MultiLineItem& multiLineItem = multiLineMap.at(multi.linesIdx[pointIdx]);
            shiftedX(pointIdx) = x(pointIdx) - multiLineItem.xMean;
            shiftedY(pointIdx) = y(pointIdx) - multiLineItem.yMean;
        }

        model->slope = (shiftedX * shiftedY).sum() / shiftedX.square().sum();
        estimator.setModel(*model);

        const double interceptCorrection = estimator.computeResiduals(x, y).mean();
        model->intercept += interceptCorrection;
        estimator.setModel(*model);
    }
} // accurate_ri
