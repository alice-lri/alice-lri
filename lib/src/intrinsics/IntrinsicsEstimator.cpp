#include "IntrinsicsEstimator.h"
#include "point/PointUtils.h"

namespace accurate_ri {
    void IntrinsicsEstimator::estimate(const PointArray &points) {
        const VerticalIntrinsicsResult vertical = verticalIntrinsicsEstimator.estimate(points);
        horizontalIntrinsicsEstimator.estimate(points, vertical);
    }
} // accurate_ri
