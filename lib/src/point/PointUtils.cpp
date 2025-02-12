#include "PointUtils.h"

#include <algorithm>
#include <cmath>

#define MIN_COORDS_EPS (1e-6 / 2)

namespace accurate_ri {
    template<PointArrayLayout T>
    std::vector<double> PointUtils::computeRanges(const PointArray<T> &points) {
        std::vector<double> ranges;
        ranges.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            const double x = points.getX(i);
            const double y = points.getY(i);
            const double z = points.getZ(i);

            ranges.emplace_back(std::sqrt(x * x + y * y + z * z));
        }

        return ranges;
    }

    template<PointArrayLayout T>
    std::vector<double> PointUtils::computePhis(const PointArray<T> &points) {
        std::vector<double> phis;
        phis.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            phis.emplace_back(std::asin(points.getZ(i) / points.getRange(i)));
        }

        return phis;
    }

    template<PointArrayLayout T>
    std::vector<double> PointUtils::computeThetas(const PointArray<T> &points) {
        std::vector<double> thetas;
        thetas.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            thetas.emplace_back(std::atan2(points.getY(i), points.getX(i)));
        }

        return thetas;
    }

    template<PointArrayLayout T>
    double PointUtils::computeCoordsEps(const PointArray<T> &points) {
        std::vector<double> sorted_x = points.getX();
        std::vector<double> sorted_y = points.getX();
        std::vector<double> sorted_z = points.getX();


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
