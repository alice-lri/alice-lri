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
            VerticalScanlinePool &scanlinePool, const PointArray &points, const ScanlineAngleBounds &angleBounds,
            const ScanlineEstimationResult &scanline, uint32_t scanlineId, const HoughCell &houghMax
        );

        bool simpleShouldKeep(
            VerticalScanlinePool &scanlinePool, const ScanlineAngleBounds &angleBounds,
            const ScanlineEstimationResult &scanline, uint32_t scanlineId, const HoughCell &houghMax
        );

    private:
        ScanlineConflictsResult evaluateScanlineConflicts(
            const VerticalScanlinePool &scanlinePool, const ScanlineAngleBounds &angleBounds,
            const ScanlineEstimationResult &scanline, uint32_t scanlineId, const HoughCell &houghMax
        );

        static ScanlineIntersectionInfo computeScanlineIntersectionInfo(
            const VerticalScanlinePool &scanlinePool, const ScanlineAngleBounds &angleBounds,
            const ScanlineEstimationResult &scanline, uint32_t scanlineId
        );
    };
} // accurate_ri
