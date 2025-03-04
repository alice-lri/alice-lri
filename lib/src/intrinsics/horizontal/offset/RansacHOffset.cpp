#include "RansacHOffset.h"
#include <cstdint>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "point/PointArray.h"

namespace accurate_ri {

    void RansacHOffset::computeOffset(PointArray& points, const uint32_t scanlineId, const uint32_t resolution) {
        auto diffToIdeal = HorizontalMath::computeDiffToIdeal(points.getThetas(), resolution, false);
        double residualThreshold = points.getCoordsEps() * 1.5;

    }
} // accurate_ri