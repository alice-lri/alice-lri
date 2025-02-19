#include "PointUtils.h"

#include <algorithm>
#include <cmath>

#define MIN_COORDS_EPS (1e-6 / 2)

namespace accurate_ri {

    double PointUtils::computeCoordsEps(const PointArray &points) {
        Eigen::ArrayXd sortedX = points.getX();
        Eigen::ArrayXd sortedY = points.getY();
        Eigen::ArrayXd sortedZ = points.getZ();

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
