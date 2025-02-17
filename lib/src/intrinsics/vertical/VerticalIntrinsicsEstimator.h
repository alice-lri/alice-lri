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

    public:
        void estimate(const PointArray &points);

    private:
        void initHough(const PointArray &points);

        static VerticalBounds computeErrorBounds(const PointArray &points, double offset);

        ScanlineLimits computeScanlineLimits(
            const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
            double invRangesShift
        ) const;

        void tryFitScanline(
            const PointArray &points, const OffsetAngle &scanlineAttributes, VerticalBounds &errorBounds,
            ScanlineLimits &scanlineLimits
        ) const;

        static LinearFitResult performLinearFit(
            const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &bounds
        );
    };
} // namespace accurate_ri
