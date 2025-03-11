#include "CustomEstimator.h"
#include <Eigen/Core>
#include "math/Stats.h"

namespace accurate_ri {
    const Stats::LRResult &CustomEstimator::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        lrResult = Stats::simpleLinearRegression(x, y);
        return lrResult;
    }

    const Eigen::ArrayXd &CustomEstimator::computeResiduals(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        const auto &multi = computeMultiLine(x, y);
        residuals = (multi.distances - multi.linesIdx.cast<double>() * thetaStep).abs();

        return residuals;
    }

    const MultiLineResult &CustomEstimator::computeMultiLine(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        multiLineResult.distances = y - (lrResult.slope * x + lrResult.intercept);
        multiLineResult.linesIdx = (multiLineResult.distances / thetaStep).round().cast<int>();

        return multiLineResult;
    }
} // accurate_ri
