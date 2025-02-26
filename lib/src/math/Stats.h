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

    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds);
}
