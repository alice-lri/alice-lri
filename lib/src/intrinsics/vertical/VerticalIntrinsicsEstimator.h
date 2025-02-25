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
        Eigen::ArrayXi pointsScanlinesIds;

        std::unordered_multimap<uint32_t, uint32_t> reverseScanlinesDependencyMap;
        std::unordered_map<uint64_t, HashToConflictValue> hashesToConflictsMap;

        int64_t unassignedPoints = 0;

    public:
        VerticalIntrinsicsResult estimate(const PointArray &points);

    private:
        void initHough(const PointArray &points);

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

        static LinearFitResult performLinearFit(
            const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &bounds
        );

        HeuristicScanline computeHeuristicScanline(double invRangesMean, double phisMean) const;

        ScanlineIntersectionInfo computeScanlineIntersectionInfo(
            const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId
        );

        bool performScanlineConflictResolution(
            const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId,
            const HoughCell &houghMax
        );

        ScanlineConflictsResult evaluateScanlineConflicts(
            const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, uint32_t scanlineId,
            const HoughCell &houghMax
        );
    };
} // namespace accurate_ri
