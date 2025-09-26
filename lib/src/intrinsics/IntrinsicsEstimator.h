#pragma once
#include "alice_lri/Structs.hpp"
#include "horizontal/HorizontalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include "vertical/VerticalIntrinsicsEstimator.h"

namespace alice_lri {
    class IntrinsicsEstimator {

    public:
        static Intrinsics estimate(const PointArray &points);
        static IntrinsicsDetailed debugEstimate(const PointArray &points);

    private:
        static Scanline makeScanline(const VerticalScanline &vertical, const HorizontalScanline &horizontal);
        static ScanlineDetailed makeDebugScanline(const VerticalScanline &vertical, const HorizontalScanline &horizontal);
    };
}
