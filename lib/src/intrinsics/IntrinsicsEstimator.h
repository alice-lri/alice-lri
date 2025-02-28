#pragma once
#include "intrinsics/horizontal/HorizontalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include "vertical/VerticalIntrinsicsEstimator.h"


namespace accurate_ri {

class IntrinsicsEstimator {
private:
    HorizontalIntrinsicsEstimator horizontalIntrinsicsEstimator;
    VerticalIntrinsicsEstimator verticalIntrinsicsEstimator;

public:
    void estimate(const PointArray& points);
};
} // accurate_ri
