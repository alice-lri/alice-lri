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
            const double diffX = sortedX[i] - sortedX[i - 1];
            minDiff = diffX > 0 ? std::min(minDiff, diffX) : minDiff;

            const double diffY = sortedY[i] - sortedY[i - 1];
            minDiff = diffY > 0 ? std::min(minDiff, diffY) : minDiff;

            const double diffZ = sortedZ[i] - sortedZ[i - 1];
            minDiff = diffZ > 0 ? std::min(minDiff, diffZ) : minDiff;
        }

        return std::max(minDiff / 2, MIN_COORDS_EPS);
    }
} // accurate_ri
