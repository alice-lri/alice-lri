#include "IntrinsicsEstimator.h"

#include <accurate_ri.h>

#include "point/PointUtils.h"
#include "utils/Logger.h"

namespace accurate_ri {
    void IntrinsicsEstimator::estimate(const PointArray &points) {
        const VerticalIntrinsicsResult vertical = verticalIntrinsicsEstimator.estimate(points);
        horizontalIntrinsicsEstimator.estimate(points, vertical);
    }
} // accurate_ri
