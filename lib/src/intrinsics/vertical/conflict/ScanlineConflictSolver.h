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
        static ScanlineConflictsResult evaluateScanlineConflicts(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static ScanlineIntersectionInfo computeScanlineIntersectionInfo(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );
    };
} // accurate_ri
