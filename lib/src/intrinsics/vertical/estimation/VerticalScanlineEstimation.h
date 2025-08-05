#pragma once
#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri::VerticalScanlineEstimation {
    std::optional<ScanlineEstimationResult> estimateScanline(
        const PointArray &points, const VerticalScanlinePool &scanlinePool,
        const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    );
} // accurate_ri
