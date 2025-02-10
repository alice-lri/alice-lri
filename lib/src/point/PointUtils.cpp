#include "PointUtils.h"
#include <cmath>

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
} // accurate_ri
