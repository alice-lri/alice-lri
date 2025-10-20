#pragma once
#include <span>
#include <Eigen/Core>

namespace alice_lri::Stats {
    double weightedMedian(std::span<const double> values, std::span<const int32_t> weights);
}
