#pragma once
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "point/PointArray.h"

namespace accurate_ri::VerticalScanlineLimits {

    VerticalBounds computeErrorBounds(const PointArray &points, double offset);

    ScanlineLimits computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const double offset, const double angle,
        const VerticalMargin &margin
    );

} // accurate_ri

