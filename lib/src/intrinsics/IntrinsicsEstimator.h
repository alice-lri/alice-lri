#pragma once
#include "accurate_ri/public_structs.hpp"
#include "horizontal/HorizontalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include "vertical/VerticalIntrinsicsEstimator.h"

namespace accurate_ri {
    class IntrinsicsEstimator {
    private:
        HorizontalIntrinsicsEstimator horizontalIntrinsicsEstimator;
        VerticalIntrinsicsEstimator verticalIntrinsicsEstimator;

    public:
        Intrinsics estimate(const PointArray &points);

    private:
        static Scanline makeScanline(const VerticalScanline &vertical, const HorizontalScanline &horizontal);
    };
} // accurate_ri
