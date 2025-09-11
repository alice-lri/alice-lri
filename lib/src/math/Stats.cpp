#include "Stats.h"
#include <numeric>
#include "utils/Timer.h"

// TODO beautify in general
namespace accurate_ri::Stats {

    template<typename T>
    T intModeImpl(const std::vector<T> &values) {
        if (values.empty()) {
            return 0;
        }

        std::unordered_map<T, int64_t> frequencies;

        for (const auto &value : values) {
            ++frequencies[value];
        }

        T mode = values[0];
        T maxCount = 0;

        for (const auto &[value, count] : frequencies) {
            if (count > maxCount) {
                maxCount = count;
                mode = value;
            }
        }

        return mode;
    }

    int64_t intMode(const std::vector<int64_t> &values) {
        return intModeImpl(values);
    }

    int32_t intMode(const std::vector<int32_t> &values) {
        return intModeImpl(values);
    }

    double mean(const std::vector<double> &values) {
        return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    }

    double weightedMean(const Eigen::ArrayXd& values, const std::span<const int32_t> weights) {
        const std::size_t n = values.size();
        if (n == 0 || weights.size() != n) {
            throw std::invalid_argument("Mismatched sizes or empty input in computeWeightedMeanSpan");
        }

        double weightedSum = 0.0;
        int32_t totalWeight = 0;

        for (std::size_t i = 0; i < n; ++i) {
            weightedSum += values[i] * weights[i];
            totalWeight += weights[i];
        }

        if (totalWeight == 0) {
            throw std::invalid_argument("Total weight cannot be zero in computeWeightedMeanSpan");
        }

        return weightedSum / totalWeight;
    }

    double weightedMedian(const std::span<const double> values, const std::span<const int32_t> weights) {
        const std::size_t n = values.size();
        if (n == 0 || weights.size() != n) {
            throw std::invalid_argument("Mismatched sizes or empty input in computeWeightedMedianSpan");
        }

        std::vector<std::size_t> indices(n);
        std::iota(indices.begin(), indices.end(), 0);

        std::ranges::sort(indices, [&](const std::size_t a, const std::size_t b) {
            return values[a] < values[b];
        });

        const double totalWeight = std::accumulate(weights.begin(), weights.end(), 0.0);
        double cumulativeWeight = 0.0;

        for (const std::size_t idx : indices) {
            cumulativeWeight += weights[idx];
            if (cumulativeWeight >= totalWeight / 2.0) {
                return values[idx];
            }
        }

        return values[indices.back()];
    }
}
