#pragma once
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "point/PointArray.h"

namespace alice_lri::VerticalScanlineLimits {

    VerticalBounds computeErrorBounds(const PointArray &points, double offset);

    ScanlineLimits computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, double offset, double angle,
        const VerticalMargin &margin
    );

}

