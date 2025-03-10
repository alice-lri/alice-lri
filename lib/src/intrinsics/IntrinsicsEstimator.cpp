#include "IntrinsicsEstimator.h"
#include "utils/Logger.h"

namespace accurate_ri {
    IntrinsicsResult IntrinsicsEstimator::estimate(const PointArray &points) {
        VerticalIntrinsicsResult vertical = verticalIntrinsicsEstimator.estimate(points);
        HorizontalIntrinsicsResult horizontal = horizontalIntrinsicsEstimator.estimate(points, vertical);

        return IntrinsicsResult {
            .vertical = std::move(vertical),
            .horizontal = std::move(horizontal),
        };
    }
} // accurate_ri
