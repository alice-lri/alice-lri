#pragma once
#include <memory>
#include "hough/HoughTransform.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalIntrinsicsEstimator {
    private:
        std::unique_ptr<VerticalScanlinePool> scanlinePool = nullptr;

    public:
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

        void writeToJson(const VerticalIntrinsicsResult &result);
    };
} // namespace accurate_ri
