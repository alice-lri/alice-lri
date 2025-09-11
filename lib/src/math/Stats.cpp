#include "Stats.h"
#include <numeric>
#include "utils/Timer.h"

namespace accurate_ri::Stats {
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
