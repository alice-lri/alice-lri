#pragma once
#include "intrinsics/vertical/VerticalStructs.h"
#include "point/PointArray.h"
namespace accurate_ri {

class HorizontalIntrinsicsEstimator {
   public:
    void estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);
};

}  // namespace accurate_ri
