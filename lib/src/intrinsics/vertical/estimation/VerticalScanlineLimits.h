#pragma once
#include "intrinsics/vertical/VerticalStructs.h"
#include "point/PointArray.h"

namespace accurate_ri::VerticalScanlineLimits {

    VerticalBounds computeErrorBounds(const PointArray &points, double offset);

    ScanlineLimits computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const OffsetAngleMargin &margin
    );

} // accurate_ri

