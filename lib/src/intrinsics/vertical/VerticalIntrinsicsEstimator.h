#pragma once
#include <memory>
#include "intrinsics/vertical/conflict/ScanlineConflictSolver.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    struct Candidate {
        std::optional<HoughScanlineEstimation> hough = std::nullopt;
        std::optional<EndReason> endReason = std::nullopt;
        bool valid = false;
    };

    struct RefinedCandidate {
        ScanlineEstimationResult scanline;
    };

    class VerticalIntrinsicsEstimator {
    private:
        // TODO maybe avoid this
        std::unique_ptr<VerticalScanlinePool> scanlinePool = nullptr;
        ScanlineConflictSolver conflictSolver;

    public:
        VerticalIntrinsicsResult estimate(const PointArray &points);

    private:
        void init(const PointArray &points);

        Candidate findCandidate(int64_t iteration) const;

        std::optional<RefinedCandidate> refineCandidate(
            int64_t iteration, const PointArray &points, const Candidate& candidate
        ) const;

        VerticalIntrinsicsResult extractResult(
            int64_t iteration, const PointArray &points, EndReason endReason
        ) const;

        static VerticalBounds computeErrorBounds(const PointArray &points, double offset);

        [[nodiscard]] ScanlineLimits computeScanlineLimits(
            const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
            const OffsetAngleMargin &margin, double invRangesShift
        ) const;

        std::optional<ScanlineEstimationResult> estimateScanline(
            const PointArray &points, const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
        ) const;

        [[nodiscard]] ScanlineFitResult tryFitScanline(
            const PointArray &points, const VerticalBounds &errorBounds,
            const ScanlineLimits &scanlineLimits
        ) const;

        HeuristicScanline computeHeuristicScanline(double invRangesMean, double phisMean) const;
    };
} // namespace accurate_ri
