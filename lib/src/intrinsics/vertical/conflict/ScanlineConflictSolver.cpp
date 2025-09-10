#include "ScanlineConflictSolver.h"

#include <queue>
#include <functional>

#include "intrinsics/vertical/conflict/IntersectionCalculator.h"
#include "intrinsics/vertical/helper/VerticalLogging.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/logger/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    bool ScanlineConflictSolver::performScanlineConflictResolution(
        VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate
    ) {
        const ScanlineConflicts conflicts = evaluateConflicts(scanlinePool, candidate);

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

    ScanlineConflicts ScanlineConflictSolver::evaluateConflicts(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        auto intersectionFlags = IntersectionCalculator::computeIntersectionFlags(scanlinePool, candidate);
        if (!intersectionFlags.anyIntersection()) {
            return {
                .shouldReject = false,
                .conflictingScanlines = std::vector<uint32_t>()
            };
        }

        auto intersectionInfo = IntersectionCalculator::computeIntersectionInfo(scanlinePool, std::move(intersectionFlags));
        VerticalLogging::logIntersectionProblem(intersectionInfo, candidate);

        constexpr double inf = std::numeric_limits<double>::infinity();
        const double minConflictingUncertainty = std::ranges::min(intersectionInfo.conflictingUncertainties) - 1e-6;

        if (candidate.scanline.uncertainty == inf && minConflictingUncertainty == inf) {
            return rejectCandidateIfEmpiricalIntersection(std::move(intersectionInfo));
        }

        if (candidate.scanline.uncertainty >= minConflictingUncertainty) {
            return rejectCandidate(candidate, intersectionInfo);
        }

        return rejectConflicting(std::move(intersectionInfo));
    }


    ScanlineConflicts ScanlineConflictSolver::rejectCandidateIfEmpiricalIntersection(
        ScanlineIntersectionInfo &&intersection
    ) {
        LOG_INFO(
            "New uncertainty is infinite, but so are the conflicting scanlines uncertainties. ",
            "Intersects other empirical: ", intersection.flags.empiricalIntersection? "True": "False", "."
        );

        const bool reject = intersection.flags.empiricalIntersection;

        return {
            .shouldReject = reject,
            .conflictingScanlines = reject? std::move(intersection.conflictingIds) : std::vector<uint32_t>(),
        };
    }

    ScanlineConflicts ScanlineConflictSolver::rejectCandidate(
        const VerticalScanlineCandidate &candidate, const ScanlineIntersectionInfo &intersection
    ) {
        std::vector<uint32_t> betterScanlineIds;
        betterScanlineIds.reserve(intersection.conflictingIds.size());

        for (int i = 0; i < intersection.conflictingIds.size(); ++i) {
            if (candidate.scanline.uncertainty >= intersection.conflictingUncertainties[i]) {
                betterScanlineIds.emplace_back(intersection.conflictingIds[i]);
            }
        }

        LOG_INFO("New uncertainty is higher than conflicting scanlines uncertainties. Rejecting current scanline");

        return {
            .shouldReject = true,
            .conflictingScanlines = std::move(betterScanlineIds)
        };
    }

    ScanlineConflicts ScanlineConflictSolver::rejectConflicting(ScanlineIntersectionInfo &&intersection) {
        LOG_INFO("New uncertainty is lower than conflicting scanlines uncertainties. Rejecting conflicting scanlines");

        return {
            .shouldReject = false,
            .conflictingScanlines = std::move(intersection.conflictingIds)
        };
    }

    bool ScanlineConflictSolver::simpleShouldKeep(
        VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        const auto intersectionInfo = IntersectionCalculator::computeIntersectionFlags(scanlinePool, candidate);
        const bool conflicting = intersectionInfo.empiricalIntersection;

        if (conflicting) {
            scanlinePool.invalidateByHash(candidate.scanline.hough.cell.hash);
        }

        return !conflicting;
    }
} // accurate_ri
