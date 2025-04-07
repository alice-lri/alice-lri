#pragma once
#include <cstdint>

#include "math/Stats.h"

namespace accurate_ri {
    struct MultiLineResult {
        Eigen::ArrayXd distances;
        Eigen::ArrayXi linesIdx;
    };

    class CustomEstimator {
    private:
        const double thetaStep;
        Stats::LRResult lrResult = Stats::LRResult();
        MultiLineResult multiLineResult = MultiLineResult();
        Eigen::ArrayXd residuals = Eigen::ArrayXd();

    public:
        explicit CustomEstimator(const double thetaStep) : thetaStep(thetaStep) {}

        const Stats::LRResult &fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        const Eigen::ArrayXd &computeResiduals(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        const MultiLineResult &computeMultiLine(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        [[nodiscard]] MultiLineResult getLastMultiLine() const {
            return multiLineResult;
        }

        void setModel(const Stats::LRResult &lrResult) {
            this->lrResult = lrResult;
        }
    };
} // accurate_ri
