#include "IntrinsicsEstimator.h"
#include "point/PointUtils.h"

namespace accurate_ri {
    void IntrinsicsEstimator::estimate(const PointArray &points) {
        verticalIntrinsicsEstimator.estimate(points);
        horizontalIntrinsicsEstimator.estimate(points);
    }
} // accurate_ri
