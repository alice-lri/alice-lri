#pragma once
#include <optional>

#include "intrinsics/vertical/estimation/VerticalScanlineEstimationStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalScanlineEstimator {
    private:
        int32_t ciTooWideState = 0;

        enum class FitConvergenceState {
            INITIAL = 0, CONVERGED = 1, CONFIRMED = 2,
        };

    public:
        std::optional<VerticalScanlineEstimation> estimate(
            const PointArray &points, const VerticalScanlinePool &scanlinePool,
            const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
        );

    private:
        ScanlineFitResult tryFitScanline(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const VerticalBounds &errorBounds, const
            ScanlineLimits &scanlineLimits
        );

        std::optional<VerticalScanlineEstimation> performStatisticalFit(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const VerticalBounds &errorBounds,
            const ScanlineLimits &scanlineLimits
        );

        static std::optional<Eigen::ArrayXi> refinePointsToFitIndices(
            const PointArray &points, const ScanlineLimits &scanlineLimits, FitConvergenceState state
        );

        static Stats::WLSResult fitScanline(
            const PointArray &points, const Eigen::ArrayXi &pointsToFitIndices, const VerticalBounds &errorBounds,
            FitConvergenceState state
        );

        bool verifyConfidenceIntervals(const Stats::WLSResult &fitResult);

        static ScanlineLimits computeLimits(
            const PointArray &points, const VerticalScanlinePool &scanlinePool, const Stats::WLSResult &fitResult,
            const VerticalBounds &errorBounds
        );

        static FitConvergenceState computeConvergenceState(
            const Eigen::ArrayX<bool> &oldMask, const Eigen::ArrayX<bool> &newMask, FitConvergenceState oldState
        );

        static ScanlineFitResult makeFitResult(
            const ScanlineLimits &currentScanlineLimits, const std::optional<Stats::WLSResult> &fitResult,
            FitConvergenceState convergenceState, bool validCi
        );

        static std::optional<VerticalScanlineEstimation> scanlineFitToEstimation(
            const PointArray &points, ScanlineFitResult& scanlineFit
        );
    };
} // accurate_ri
