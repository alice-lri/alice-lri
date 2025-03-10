#pragma once
#include <iosfwd>
#include <unordered_map>

#include "intrinsics/horizontal/helper/HorizontalStructs.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "point/PointArray.h"
namespace accurate_ri {

class HorizontalIntrinsicsEstimator {
   public:
    HorizontalIntrinsicsResult estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);

private:
    static void updateScanlinesUseHeuristics(
        std::unordered_map<uint32_t, ScanlineHorizontalInfo> &scanlineInfoMap, const std::vector<uint32_t> &heuristicScanlines
    );

    static std::pair<int32_t, double> heuristicHAndResolution(
        const std::vector<double> &hOffsets, const std::vector<int32_t> &resolutions
    );
};

}  // namespace accurate_ri
