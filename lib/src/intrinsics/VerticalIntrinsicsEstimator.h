
#pragma once
#include "point/PointArray.h"

namespace accurate_ri {

class VerticalIntrinsicsEstimator {

public:
    template <PointArrayLayout T>
    void estimate(const PointArray<T>& points);
};

} // accurate_ri
