#pragma once
#include "HorizontalIntrinsicsEstimator.h"
#include "VerticalIntrinsicsEstimator.h"
#include "point/PointArray.h"


namespace accurate_ri {

class IntrinsicsEstimator {
private:
    HorizontalIntrinsicsEstimator horizontalIntrinsicsEstimator;
    VerticalIntrinsicsEstimator verticalIntrinsicsEstimator;

public:
    void estimate(const PointArray& points);
};
} // accurate_ri
