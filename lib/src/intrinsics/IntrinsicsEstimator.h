#pragma once
#include "alice_lri/public_structs.hpp"
#include "horizontal/HorizontalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include "vertical/VerticalIntrinsicsEstimator.h"

namespace alice_lri {
    class IntrinsicsEstimator {

    public:
        static Intrinsics estimate(const PointArray &points);
        static DebugIntrinsics debugEstimate(const PointArray &points);

    private:
        static Scanline makeScanline(const VerticalScanline &vertical, const HorizontalScanline &horizontal);
        static DebugScanline makeDebugScanline(const VerticalScanline &vertical, const HorizontalScanline &horizontal);
    };
}
