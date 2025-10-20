#pragma once
#include "intrinsics/vertical/conflict/ScanlineConflictSolver.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace alice_lri {
    class VerticalIntrinsicsEstimator {

    public:
        static VerticalIntrinsicsEstimation estimate(const PointArray &points);

    private:
        static VerticalScanlinePool init(const PointArray &points);

        static VerticalScanlineHoughCandidate findCandidate(const VerticalScanlinePool &scanlinePool, int64_t iteration);

        static std::optional<VerticalScanlineEstimation> estimateScanline(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const HoughScanlineEstimation &hough
        );

        static VerticalIntrinsicsEstimation extractResult(
            int64_t iteration, const PointArray &points, VerticalScanlinePool &scanlinePool, EndReason endReason
        );

        static VerticalScanline makeVerticalScanline(
            uint32_t currentScanlineId, const VerticalScanlineHoughCandidate &houghCandidate,
            const std::optional<VerticalScanlineEstimation> &estimation, const ScanlineAngleBounds &angleBounds
        );
    };
}
