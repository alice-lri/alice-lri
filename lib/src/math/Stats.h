#pragma once
#include <optional>
#include <span>
#include <Eigen/Core>

namespace accurate_ri::Stats {

    int64_t intMode(const std::vector<int64_t> &values);

    int32_t intMode(const std::vector<int32_t> &values);

    double mean(const std::vector<double> &values);

    double weightedMean(const Eigen::ArrayXd& values, std::span<const int32_t> weights);

    double weightedMedian(std::span<const double> values, std::span<const int32_t> weights);
}
