#pragma once
#include "point/PointArray.h"
namespace accurate_ri {

class HorizontalIntrinsicsEstimator {
   public:
    void estimate(const PointArray& points);
};

}  // namespace accurate_ri
