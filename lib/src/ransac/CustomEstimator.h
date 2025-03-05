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
        const uint32_t resolution;
        Stats::LRResult lrResult = Stats::LRResult();
        MultiLineResult multiLineResult = MultiLineResult();
        Eigen::ArrayXd residuals = Eigen::ArrayXd();

    public:
        explicit CustomEstimator(const uint32_t resolution) : resolution(resolution) {}

        const Stats::LRResult &fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        const Eigen::ArrayXd &computeResiduals(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        const MultiLineResult &computeMultiLine(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
    };
} // accurate_ri
