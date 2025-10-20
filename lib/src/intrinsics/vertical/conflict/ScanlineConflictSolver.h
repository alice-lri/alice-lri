#pragma once

#include <unordered_map>
#include "intrinsics/vertical/conflict/ScanlineConflictStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineEstimationStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace alice_lri {
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
        void rejectScanline(
            VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate, const ScanlineConflicts &conflicts
        );

        void rejectConflictingScanlines(
            VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate,
            const ScanlineConflicts &conflicts
        );

        void restorePreviouslyRejectedByConflictingId(
            std::vector<std::pair<uint64_t, int64_t>> &hashesToRestore, uint32_t conflictingId
        );

        void markAsRejectedByConflictingIds(const HoughCell &rejected, const std::span<const uint32_t> &conflictingIds);

    };
}
