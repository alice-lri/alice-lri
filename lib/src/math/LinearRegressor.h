#pragma once
#include <optional>
#include <Eigen/Core>


namespace alice_lri {
    struct WLSResult {
        double slope;
        double intercept;
        double slopeVariance;
        double interceptVariance;
        double logLikelihood;
        Eigen::Array2d slopeCi;
        Eigen::Array2d interceptCi;
    };

    struct LRResult {
        double slope = 0;
        double intercept = 0;
        std::optional<double> mse;

        LRResult() = default;
        LRResult(const double slope, const double intercept) : slope(slope), intercept(intercept), mse(std::nullopt) {}
        LRResult(const double slope, const double intercept, const std::optional<double> &mse)
            : slope(slope), intercept(intercept), mse(mse) {}
    };

    namespace LinearRegressor {
        LRResult fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, bool computeMse = false);
        WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds);
    }
}
