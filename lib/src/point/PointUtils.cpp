#include "PointUtils.h"

#include <algorithm>
#include <cmath>

#define MIN_COORDS_EPS (1e-6 / 2)

namespace accurate_ri {

    std::vector<double> PointUtils::computeRanges(const std::vector<double> &x,
                                                  const std::vector<double> &y,
                                                  const std::vector<double> &z) {
        std::vector<double> ranges;
        ranges.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            ranges.push_back(std::sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]));
        }
        return ranges;
    }

    std::vector<double> PointUtils::computePhis(const std::vector<double> &x,
                                                const std::vector<double> &z,
                                                const std::vector<double> &ranges) {
        std::vector<double> phis;
        phis.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            phis.push_back(std::asin(z[i] / ranges[i]));
        }
        return phis;
    }

    std::vector<double> PointUtils::computeThetas(const std::vector<double> &x,
                                                  const std::vector<double> &y) {
        std::vector<double> thetas;
        thetas.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            thetas.push_back(std::atan2(y[i], x[i]));
        }
        return thetas;
    }

    double PointUtils::computeCoordsEps(const std::vector<double> &x, const std::vector<double> &y,
                                        const std::vector<double> &z) {
        std::vector<double> sorted_x = x;
        std::vector<double> sorted_y = y;
        std::vector<double> sorted_z = z;

        std::ranges::sort(sorted_x);
        std::ranges::sort(sorted_y);
        std::ranges::sort(sorted_z);

        double min_diff = std::numeric_limits<double>::max();

        for (size_t i = 1; i < sorted_x.size(); i++) {
            min_diff = std::min(min_diff, sorted_x[i] - sorted_x[i - 1]);
            min_diff = std::min(min_diff, sorted_y[i] - sorted_y[i - 1]);
            min_diff = std::min(min_diff, sorted_z[i] - sorted_z[i - 1]);
        }

        return std::max(min_diff, MIN_COORDS_EPS);
    }
} // accurate_ri
