#pragma once
#include <eigen3/Eigen/Core>

namespace accurate_ri::Stats {
    struct WLSResult {
        double slope;
        double intercept;
        double slopeVariance;
        double interceptVariance;
        double aic;
        Eigen::Array2d slopeCi;
        Eigen::Array2d interceptCi;
    };

    struct LRResult {
        double slope;
        double intercept;
    };

    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds);
    LRResult simpleLinearRegression(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
    int64_t intMode(const std::vector<int64_t> &values);
    int32_t intMode(const std::vector<int32_t> &values);
}
