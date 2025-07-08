#pragma once
#include <optional>
#include <span>
#include <Eigen/Core>

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
        std::optional<double> mse;
    };

    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds);

    LRResult simpleLinearRegression(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, bool computeMse = false);

    int64_t intMode(const std::vector<int64_t> &values);

    int32_t intMode(const std::vector<int32_t> &values);

    double weightedMedian(std::span<const double> values, std::span<const int32_t> weights);

    double circularLinearCorr(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, double period = 2 * M_PI);
}
