#include "IntrinsicsEstimator.h"
#include "utils/logger/Logger.h"

namespace accurate_ri {
    Intrinsics IntrinsicsEstimator::estimate(const PointArray &points) {
        VerticalIntrinsicsEstimation vertical = verticalIntrinsicsEstimator.estimate(points);
        HorizontalIntrinsicsResult horizontal = horizontalIntrinsicsEstimator.estimate(points, vertical);

        // return Intrinsics {
        //     .vertical = std::move(vertical),
        //     .horizontal = std::move(horizontal),
        // }; // TODO RECOVER THIS
        return Intrinsics();
    }
} // accurate_ri
