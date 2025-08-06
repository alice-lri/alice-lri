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
        double logLikelihood;
        Eigen::Array2d slopeCi;
        Eigen::Array2d interceptCi;
    };

    struct LRResult {
        double slope;
        double intercept;
        std::optional<double> mse;

        LRResult() = default;

        LRResult(const double slope, const double intercept) : slope(slope), intercept(intercept), mse(std::nullopt) {}

        LRResult(double slope, double intercept, const std::optional<double> &mse) : slope(slope),
            intercept(intercept),
            mse(mse) {}
    };

    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds);

    LRResult linearRegression(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, bool computeMse = false);

    int64_t intMode(const std::vector<int64_t> &values);

    int32_t intMode(const std::vector<int32_t> &values);

    double mean(const std::vector<double> &values);

    double weightedMean(const Eigen::ArrayXd& values, std::span<const int32_t> weights);

    double weightedMedian(std::span<const double> values, std::span<const int32_t> weights);
}
