#pragma once
#include <memory>
#include "intrinsics/vertical/conflict/ScanlineConflictSolver.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    struct VerticalScanlineHoughCandidate {
        std::optional<HoughScanlineEstimation> estimation = std::nullopt;
        std::optional<EndReason> endReason = std::nullopt;
        bool valid = false;
    };

    class VerticalIntrinsicsEstimator {
    private:
        // TODO maybe avoid this
        // TODO consider removing these and making it stateless
        std::unique_ptr<VerticalScanlinePool> scanlinePool = nullptr;
        ScanlineConflictSolver conflictSolver;

    public:
        VerticalIntrinsicsEstimation estimate(const PointArray &points);

    private:
        void init(const PointArray &points);

        VerticalScanlineHoughCandidate findCandidate(int64_t iteration) const;

        std::optional<VerticalScanlineEstimation> estimateScanline(
            const PointArray &points, const HoughScanlineEstimation &hough
        ) const;

        VerticalIntrinsicsEstimation extractResult(
            int64_t iteration, const PointArray &points, EndReason endReason
        ) const;

        static VerticalScanline makeVerticalScanline(
            uint32_t currentScanlineId, const VerticalScanlineHoughCandidate &candidate,
            const std::optional<VerticalScanlineEstimation> &refinedCandidate, const ScanlineAngleBounds &angleBounds
        );
    };
} // namespace accurate_ri
