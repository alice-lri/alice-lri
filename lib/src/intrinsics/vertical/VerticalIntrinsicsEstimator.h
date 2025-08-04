#pragma once
#include <memory>
#include "hough/HoughTransform.h"
#include "intrinsics/vertical/conflict/ScanlineConflictSolver.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalIntrinsicsEstimator {
    private:
        std::unique_ptr<VerticalScanlinePool> scanlinePool = nullptr;
        ScanlineConflictSolver conflictSolver;

    public:
        void init(const PointArray &points);

        void logHoughInfo(int64_t iteration, const HoughCell &houghMax);

        void logScanlineAssignation(
            const ScanlineInfo &scanline
        );

        VerticalIntrinsicsResult estimate(const PointArray &points);

    private:
        void initScanlinePool(const PointArray &points);

        static VerticalBounds computeErrorBounds(const PointArray &points, double offset);

        [[nodiscard]] ScanlineLimits computeScanlineLimits(
            const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
            const OffsetAngleMargin &margin, double invRangesShift
        ) const;

        std::optional<ScanlineEstimationResult> estimateScanline(
            const PointArray &points, const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
        );

        [[nodiscard]] ScanlineFitResult tryFitScanline(
            const PointArray &points, const VerticalBounds &errorBounds,
            const ScanlineLimits &scanlineLimits
        ) const;

        HeuristicScanline computeHeuristicScanline(double invRangesMean, double phisMean) const;
    };
} // namespace accurate_ri
