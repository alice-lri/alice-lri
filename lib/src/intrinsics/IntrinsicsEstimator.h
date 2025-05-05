#pragma once
#include "accurate_ri/public_structs.hpp"
#include "intrinsics/horizontal/HorizontalIntrinsicsEstimator.h"
#include "intrinsics/horizontal/coarsetofine/CoarseToFineHorizontalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include "vertical/VerticalIntrinsicsEstimator.h"


namespace accurate_ri {

class IntrinsicsEstimator {
private:
    HorizontalIntrinsicsEstimator horizontalIntrinsicsEstimator;
    VerticalIntrinsicsEstimator verticalIntrinsicsEstimator;

public:
    IntrinsicsResult estimate(const PointArray &points);
};
} // accurate_ri
