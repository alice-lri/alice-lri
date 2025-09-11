#pragma once
#include <optional>
#include <span>
#include <Eigen/Core>

namespace accurate_ri::Stats {
    double weightedMedian(std::span<const double> values, std::span<const int32_t> weights);
}
