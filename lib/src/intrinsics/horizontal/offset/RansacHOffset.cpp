#include "RansacHOffset.h"
#include <accurate_ri/accurate_ri.hpp>
#include <cstdint>
#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "ransac/CustomRansac.h"

namespace accurate_ri {
    std::optional<RansacHOffsetResult> RansacHOffset::computeOffset(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const int32_t resolution
    ) {
        CustomRansac ransac(100, resolution);
        const std::optional<CustomRansacResult> ransacResult = ransac.fit(scanlineArray, scanlineIdx);

        return ransacResult.has_value()
                   ? std::make_optional<RansacHOffsetResult>(
                       {
                           .offset = ransacResult->model.slope,
                           .loss = ransacResult->loss * resolution,
                       }
                   )
                   : std::nullopt;
    }
} // accurate_ri
