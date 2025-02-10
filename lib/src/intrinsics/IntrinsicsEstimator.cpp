#include "IntrinsicsEstimator.h"
#include "../point/PointUtils.h"

namespace accurate_ri {
    void IntrinsicsEstimator::estimate(const std::vector<double> &x,
                                       const std::vector<double> &y,
                                       const std::vector<double> &z) {
        const double coordsEps = PointUtils::computeCoordsEps(x, y, z);
        const std::vector<double> ranges = PointUtils::computeRanges(x, y, z);
        const std::vector<double> phis = PointUtils::computePhis(x, z, ranges);


    }
} // accurate_ri
