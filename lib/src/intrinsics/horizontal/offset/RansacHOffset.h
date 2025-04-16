#pragma once
#include <optional>

#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "point/PointArray.h"

namespace accurate_ri {
    struct RansacHOffsetResult {
        double offset;
        double loss;
    };

    class RansacHOffset {
    public:
        static std::optional<RansacHOffsetResult> computeOffset(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t resolution
        );
    };
} // accurate_ri
