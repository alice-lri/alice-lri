#pragma once

#include <unordered_map>
#include "intrinsics/vertical/conflict/ScanlineConflictStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineEstimationStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace accurate_ri {
    class ScanlineConflictSolver {
    private:
        std::unordered_map<uint64_t, HashToConflictValue> hashesToConflictsMap;

    public:
        bool performScanlineConflictResolution(
            VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate
        );

        static bool simpleShouldKeep(
            VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

    private:
        static ScanlineConflictsResult evaluateConflicts(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static ScanlineIntersectionFlags computeIntersectionFlags(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static Eigen::ArrayX<bool> computeEmpiricalIntersections(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static Eigen::ArrayX<bool> computeTheoreticalIntersections(
           const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
       );

        static ScanlineIntersectionInfo computeIntersectionInfo(
            const VerticalScanlinePool &scanlinePool, ScanlineIntersectionFlags &&flags
        );

        static ScanlineConflictsResult rejectCandidateIfEmpiricalIntersection(
            ScanlineIntersectionInfo &&intersection
        );

        static ScanlineConflictsResult rejectCandidate(
            const VerticalScanlineCandidate &candidate, const ScanlineIntersectionInfo &intersection
        );

        static ScanlineConflictsResult rejectConflicting(ScanlineIntersectionInfo &&intersection);
    };
} // accurate_ri
