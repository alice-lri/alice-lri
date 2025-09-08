#pragma once
#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalScanlineEstimator {
    public:
        static std::optional<ScanlineEstimationResult> performHeuristicFit(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
        );

        static std::optional<ScanlineEstimationResult> estimateScanline(
            const PointArray &points, const VerticalScanlinePool &scanlinePool,
            const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
        );

    private:
        static ScanlineFitResult tryFitScanline(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const VerticalBounds &errorBounds, const
            ScanlineLimits &scanlineLimits
        );

        static std::optional<ScanlineEstimationResult> performStatisticalFit(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const VerticalBounds &errorBounds,
            const ScanlineLimits &scanlineLimits
        );

        static HeuristicScanline computeHeuristicScanline(
            const VerticalScanlinePool &scanlinePool, double invRangesMean, double phisMean
        );
    };
} // accurate_ri
