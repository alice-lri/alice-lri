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
        // TODO consider removing these and making it stateless
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
    };
} // namespace accurate_ri
