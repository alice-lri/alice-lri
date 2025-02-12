#pragma once
#include <vector>

#include "point/PointArray.h"
#include "point/PointUtils.h"

namespace accurate_ri {

class IntrinsicsEstimator {

public:
    template <PointArrayLayout T>
    void estimate(const PointArray<T>& points);
};
} // accurate_ri
