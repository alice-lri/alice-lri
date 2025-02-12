#include "IntrinsicsEstimator.h"
#include "point/PointUtils.h"

namespace accurate_ri {
    template<PointArrayLayout T>
    void IntrinsicsEstimator::estimate(const PointArray<T> &points) {
        const double coordsEps = PointUtils::computeCoordsEps(points);
        const std::vector<double> ranges = PointUtils::computeRanges(points);
        const std::vector<double> phis = PointUtils::computePhis(points);
    }
} // accurate_ri
