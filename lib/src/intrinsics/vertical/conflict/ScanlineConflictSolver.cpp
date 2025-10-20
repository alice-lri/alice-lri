#include "ScanlineConflictSolver.h"

#include <queue>
#include <functional>

#include "intrinsics/vertical/conflict/ScanlineConflictEvaluator.h"
#include "intrinsics/vertical/helper/VerticalLogging.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/logger/Logger.h"
#include "utils/Utils.h"

namespace alice_lri {
    bool ScanlineConflictSolver::performScanlineConflictResolution(
        VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate
    ) {
        const ScanlineConflicts conflicts = ScanlineConflictEvaluator::evaluateConflicts(scanlinePool, candidate);

        if (conflicts.shouldReject) {
            rejectScanline(scanlinePool, candidate, conflicts);
            return false;
        }

        if (conflicts.conflictingScanlines.empty()) {
            return true;
        }

        rejectConflictingScanlines(scanlinePool, points, candidate, conflicts);
        return true;
    }

    void ScanlineConflictSolver::rejectScanline(
        VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate& candidate,
        const ScanlineConflicts &conflicts
    ) {
        LOG_INFO("Scanline rejected");
        LOG_INFO("");

        const HoughCell houghMax = candidate.scanline.hough.cell;
        scanlinePool.invalidateByHash(houghMax.hash);

        if (conflicts.conflictingScanlines.empty()) {
            return;
        }

        markAsRejectedByConflictingIds(candidate.scanline.hough.cell, conflicts.conflictingScanlines);
    }

    void ScanlineConflictSolver::rejectConflictingScanlines(
        VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate,
        const ScanlineConflicts &conflicts
    ) {
        std::vector<std::pair<uint64_t, int64_t>> hashesToRestore;
        LOG_INFO("Removing scanlines: ", conflicts.conflictingScanlines);

        for (const uint32_t toRemoveId: conflicts.conflictingScanlines) {
            restorePreviouslyRejectedByConflictingId(hashesToRestore, toRemoveId);

            const std::optional<VerticalScanline> removedScanline = scanlinePool.removeScanline(points, toRemoveId);
            if (!removedScanline) {
                continue;
            }

            const auto candidateId = std::span(&candidate.scanline.id, 1);
            markAsRejectedByConflictingIds(removedScanline->hough.cell, candidateId);
        }

        for (const auto &[hash, votes] : hashesToRestore) {
            LOG_INFO("Restored hash: ", hash);
            scanlinePool.restoreByHash(hash, votes);
        }
    }

    void ScanlineConflictSolver::markAsRejectedByConflictingIds(
        const HoughCell &rejected, const std::span<const uint32_t> &conflictingIds
    ) {
        HashToConflictValue &hashToConflictValue = hashesToConflictsMap[rejected.hash];

        hashToConflictValue.conflictingScanlines.insert(conflictingIds.begin(), conflictingIds.end());
        hashToConflictValue.votes = rejected.votes;

        LOG_INFO("Added hash ", rejected.hash, " to the map");
    }

    void ScanlineConflictSolver::restorePreviouslyRejectedByConflictingId(
        std::vector<std::pair<uint64_t, int64_t>>& hashesToRestore, const uint32_t conflictingId
    ) {
        for (auto it = hashesToConflictsMap.begin(); it != hashesToConflictsMap.end();) {
            const uint64_t hash = it->first;
            HashToConflictValue &hashToConflictValue = it->second;

            hashToConflictValue.conflictingScanlines.erase(conflictingId);

            if (hashToConflictValue.conflictingScanlines.empty()) {
                hashesToRestore.emplace_back(hash, hashToConflictValue.votes);
                it = hashesToConflictsMap.erase(it);
            } else {
                ++it;
            }
        }
    }

    bool ScanlineConflictSolver::simpleShouldKeep(
        VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        const auto intersectionInfo = ScanlineConflictEvaluator::computeIntersectionFlags(scanlinePool, candidate);
        const bool conflicting = intersectionInfo.empiricalIntersection;

        if (conflicting) {
            scanlinePool.invalidateByHash(candidate.scanline.hough.cell.hash);
        }

        return !conflicting;
    }
}
