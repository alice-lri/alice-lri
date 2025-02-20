#pragma once
#include <memory>
#include <tuple>
#include <tuple>
#include <tuple>

#include "hough/HoughTransform.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalIntrinsicsEstimator {
    private:
        std::unique_ptr<HoughTransform> hough = nullptr;
        std::unordered_map<uint32_t, ScanlineInfo> scanlineInfoMap;

    public:
        VerticalIntrinsicsResult estimate(const PointArray &points);

    private:
        void initHough(const PointArray &points);

        static VerticalBounds computeErrorBounds(const PointArray &points, double offset);

        [[nodiscard]] ScanlineLimits computeScanlineLimits(
            const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
            const OffsetAngleMargin &margin, double invRangesShift
        ) const;

        [[nodiscard]] ScanlineFitResult tryFitScanline(
            const PointArray &points, const VerticalBounds &errorBounds,
            const ScanlineLimits &scanlineLimits
        ) const;

        static LinearFitResult performLinearFit(
            const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &bounds
        );

        HeuristicScanline computeHeuristicScanline(double invRangesMean, double phisMean) const;
    };
} // namespace accurate_ri
