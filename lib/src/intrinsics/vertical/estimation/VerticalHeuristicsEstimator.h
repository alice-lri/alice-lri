#pragma once
#include "intrinsics/vertical/estimation/VerticalHeuristicsStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace accurate_ri {
    class VerticalHeuristicsEstimator {
    public:
        static std::optional<ScanlineEstimationResult> estimate(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
        );

    private:
        static HeuristicScanline computeHeuristicScanline(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &limits
        );

        static ValueConfInterval computeHeuristicOffset(
            const VerticalScanlinePool &scanlinePool, double invRangesMean, double phisMean
        );

        static std::vector<uint32_t> findSupportScanlines(
            const VerticalScanlinePool &scanlinePool, double invRangesMean, double phisMean
        );

        static double computeMaxOffsetDiff(
            const VerticalScanlinePool &scanlinePool, const std::vector<uint32_t> &supportScanlineIds
        );

        static double computeMeanOffset(
            const VerticalScanlinePool &scanlinePool, const std::vector<uint32_t> &supportScanlineIds
        );

        static ValueConfInterval computeHeuristicAngle(
            const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const ValueConfInterval &offset
        );

        static OffsetAngleMargin computeHeuristicMargin(
            const VerticalScanlinePool &scanlinePool, const HeuristicScanline &scanline
        );
    };
} // accurate_ri
