#pragma once
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace accurate_ri {
    class VerticalHeuristicsEstimator {
    public:
        static std::optional<ScanlineEstimationResult> performHeuristicFit(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
        );

    private:
        static HeuristicScanline computeHeuristicScanline(
            const VerticalScanlinePool &scanlinePool, double invRangesMean, double phisMean
        );
    };
} // accurate_ri
