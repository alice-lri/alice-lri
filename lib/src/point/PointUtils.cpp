#include "PointUtils.h"

#include <algorithm>
#include <cmath>

#define MIN_COORDS_EPS (1e-6 / 2)

namespace accurate_ri {
    Eigen::VectorXd PointUtils::computeRanges(const PointArray &points) {
        Eigen::VectorXd ranges;
        ranges.resize(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            const double x = points.getX(i);
            const double y = points.getY(i);
            const double z = points.getZ(i);

            ranges[i] = std::sqrt(x * x + y * y + z * z);
        }

        return ranges;
    }

    Eigen::VectorXd PointUtils::computePhis(const PointArray &points) {
        Eigen::VectorXd phis;
        phis.resize(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            phis[i] = std::asin(points.getZ(i) / points.getRange(i));
        }

        return phis;
    }

    Eigen::VectorXd PointUtils::computeThetas(const PointArray &points) {
        Eigen::VectorXd thetas;
        thetas.resize(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            thetas[i] = std::atan2(points.getY(i), points.getX(i));
        }

        return thetas;
    }

    double PointUtils::computeCoordsEps(const PointArray &points) {
        Eigen::VectorXd sortedX = points.getX();
        Eigen::VectorXd sortedY = points.getY();
        Eigen::VectorXd sortedZ = points.getZ();

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
