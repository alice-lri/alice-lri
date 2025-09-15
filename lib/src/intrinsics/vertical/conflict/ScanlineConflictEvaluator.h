#pragma once
#include "intrinsics/vertical/conflict/ScanlineConflictStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace alice_lri {
    class ScanlineConflictEvaluator {
    public:
        static ScanlineConflicts evaluateConflicts(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static ScanlineIntersectionFlags computeIntersectionFlags(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

    private:
        static ScanlineIntersectionInfo computeIntersectionInfo(
            const VerticalScanlinePool &scanlinePool, ScanlineIntersectionFlags &&flags
        );

        static std::vector<bool> computeEmpiricalIntersections(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static std::vector<bool> computeTheoreticalIntersections(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static ScanlineConflicts rejectCandidateIfEmpiricalIntersection(ScanlineIntersectionInfo &&intersection);

        static ScanlineConflicts rejectCandidate(
            const VerticalScanlineCandidate &candidate, const ScanlineIntersectionInfo &intersection
        );

        static ScanlineConflicts rejectConflicting(ScanlineIntersectionInfo &&intersection);
    };
}