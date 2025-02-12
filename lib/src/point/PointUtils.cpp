#include "PointUtils.h"

#include <algorithm>
#include <cmath>

#define MIN_COORDS_EPS (1e-6 / 2)

namespace accurate_ri {
    std::vector<double> PointUtils::computeRanges(const PointArray &points) {
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

    std::vector<double> PointUtils::computePhis(const PointArray &points) {
        std::vector<double> phis;
        phis.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            phis.emplace_back(std::asin(points.getZ(i) / points.getRange(i)));
        }

        return phis;
    }

    std::vector<double> PointUtils::computeThetas(const PointArray &points) {
        std::vector<double> thetas;
        thetas.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            thetas.emplace_back(std::atan2(points.getY(i), points.getX(i)));
        }

        return thetas;
    }

    double PointUtils::computeCoordsEps(const PointArray &points) {
        std::vector<double> sortedX = points.getX();
        std::vector<double> sortedY = points.getY();
        std::vector<double> sortedZ = points.getZ();

        std::ranges::sort(sortedX);
        std::ranges::sort(sortedY);
        std::ranges::sort(sortedZ);

        double minDiff = std::numeric_limits<double>::max();

        for (size_t i = 1; i < sortedX.size(); i++) {
            minDiff = std::min(minDiff, sortedX[i] - sortedX[i - 1]);
            minDiff = std::min(minDiff, sortedY[i] - sortedY[i - 1]);
            minDiff = std::min(minDiff, sortedZ[i] - sortedZ[i - 1]);
        }

        return std::max(minDiff, MIN_COORDS_EPS);
    }
} // accurate_ri
