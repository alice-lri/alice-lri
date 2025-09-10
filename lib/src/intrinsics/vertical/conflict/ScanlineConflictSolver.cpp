#include "ScanlineConflictSolver.h"

#include <queue>

#include "intrinsics/vertical/helper/VerticalLogging.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/logger/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {

    bool ScanlineConflictSolver::performScanlineConflictResolution(
        VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate
    ) {
        const ScanlineConflictsResult &conflicts = evaluateConflicts(scanlinePool, candidate);

        if (conflicts.shouldReject) {
            LOG_INFO("Scanline rejected");
            LOG_INFO("");

            const HoughCell houghMax = candidate.scanline.hough.cell;
            scanlinePool.invalidateByHash(houghMax.hash);

            if (conflicts.conflictingScanlines.size() == 0) {
                return false;
            }

            HashToConflictValue &hashToConflictValue = hashesToConflictsMap[houghMax.hash];

            hashToConflictValue.conflictingScanlines.insert(
                conflicts.conflictingScanlines.begin(), conflicts.conflictingScanlines.end()
            );
            hashToConflictValue.votes = houghMax.votes;

            return false;
        }

        if (conflicts.conflictingScanlines.size() == 0) {
            return true;
        }

        LOG_INFO("Removing scanlines: ", conflicts.conflictingScanlines);
        std::vector<std::pair<uint64_t, int64_t>> hashesToRestore;

        // TODO this is trash, especially the restore by hash thing. Do properly and make sure no negative values in accumulator.
        for (const uint32_t otherId: conflicts.conflictingScanlines) {
            std::optional<VerticalScanline> removedScanline = scanlinePool.removeScanline(points, otherId);

            // Restore previously rejected scanlines due to this one
            for (auto it = hashesToConflictsMap.begin(); it != hashesToConflictsMap.end();) {
                const uint64_t hash = it->first;
                HashToConflictValue &hashToConflictValue = it->second;

                hashToConflictValue.conflictingScanlines.erase(otherId);

                if (hashToConflictValue.conflictingScanlines.empty()) {
                    hashesToRestore.emplace_back(hash, hashToConflictValue.votes);
                    it = hashesToConflictsMap.erase(it);
                } else {
                    ++it;
                }
            }

            if (!removedScanline) {
                continue;
            }

            // Store the removed line, as conflicting with current one
            hashesToConflictsMap.emplace(
                removedScanline->hough.cell.hash, HashToConflictValue{
                    .conflictingScanlines = {candidate.scanline.id},
                    .votes = removedScanline->hough.cell.votes
                }
            );

            LOG_INFO("Added hash ", removedScanline->hough.cell.hash, " to the map");
        }

        for (const auto &[hash, votes] : hashesToRestore) {
            LOG_INFO("Restored hash: ", hash);
            scanlinePool.restoreByHash(hash, votes);
        }

        return true;
    }
    
    ScanlineConflictsResult ScanlineConflictSolver::evaluateConflicts(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        ScanlineIntersectionFlags intersectionFlags = computeIntersectionFlags(scanlinePool, candidate);

        if (!intersectionFlags.anyIntersection()) {
            return {
                .shouldReject = false,
                .conflictingScanlines = Eigen::ArrayXi()
            };
        }

        auto intersectionInfo = computeIntersectionInfo(scanlinePool, std::move(intersectionFlags));
        VerticalLogging::logIntersectionProblem(intersectionInfo, candidate);

        constexpr double inf = std::numeric_limits<double>::infinity();
        const double minConflictingUncertainty = intersectionInfo.conflictingUncertainties.minCoeff() - 1e-6;

        if (candidate.scanline.uncertainty == inf && minConflictingUncertainty == inf) {
            return rejectCandidateIfEmpiricalIntersection(std::move(intersectionInfo));
        }

        if (candidate.scanline.uncertainty >= minConflictingUncertainty) {
            return rejectCandidate(candidate, intersectionInfo);
        }

        return rejectConflicting(std::move(intersectionInfo));
    }

    ScanlineIntersectionFlags ScanlineConflictSolver::computeIntersectionFlags(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        Eigen::ArrayX<bool> empiricalIntersectionMask = computeEmpiricalIntersections(scanlinePool, candidate);
        const bool empiricalIntersection = empiricalIntersectionMask.any();

        Eigen::ArrayX<bool> theoreticalIntersectionMask = computeTheoreticalIntersections(scanlinePool, candidate);
        const bool theoreticalIntersection = theoreticalIntersectionMask.any();

        return {
            .empiricalIntersectionMask = std::move(empiricalIntersectionMask),
            .theoreticalIntersectionMask = std::move(theoreticalIntersectionMask),
            .empiricalIntersection = empiricalIntersection,
            .theoreticalIntersection = theoreticalIntersection
        };
    }

    Eigen::ArrayX<bool> ScanlineConflictSolver::computeEmpiricalIntersections(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        Eigen::ArrayX<bool> result = Eigen::ArrayX<bool>::Constant(candidate.scanline.id + 1, false);
        const Eigen::ArrayXi &conflictingScanlinesIdsVerbose = scanlinePool.getScanlinesIds(candidate.limits.indices);

        for (const int32_t conflictingId: conflictingScanlinesIdsVerbose) {
            if (conflictingId >= 0) {
                result[conflictingId] = true;
            }
        }

        return result;
    }

    Eigen::ArrayX<bool> ScanlineConflictSolver::computeTheoreticalIntersections(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        Eigen::ArrayX<bool> result = Eigen::ArrayX<bool>::Constant(candidate.scanline.id + 1, true);

        const std::vector boundsLinesPointers = {&ScanlineAngleBounds::lowerLine, &ScanlineAngleBounds::upperLine};
        const ScanlineAngleBounds &angleBounds = candidate.scanline.theoreticalAngleBounds;

        for (const auto thisLinePtr: boundsLinesPointers) {
            const Interval& thisLine = angleBounds.*thisLinePtr;

            for (const auto otherLinePtr: boundsLinesPointers) {
                scanlinePool.forEachScanline([&](const VerticalScanline &otherScanline) {
                    const Interval& otherLine = otherScanline.theoreticalAngleBounds.*otherLinePtr;
                    result[otherScanline.id] = result[otherScanline.id] && otherLine.intersects(thisLine);
                });
            }
        }

        return result;
    }

    ScanlineIntersectionInfo ScanlineConflictSolver::computeIntersectionInfo(
        const VerticalScanlinePool &scanlinePool, ScanlineIntersectionFlags &&flags
    ) {
        Eigen::ArrayXi conflictingIds = Eigen::ArrayXi::Zero(flags.empiricalIntersectionMask.size());
        int j = 0;
        for (int i = 0; i < flags.empiricalIntersectionMask.size(); ++i) {
            if (flags.anyIntersection(i)) {
                conflictingIds(j++) = i;
            }
        }
        conflictingIds.conservativeResize(j);

        Eigen::ArrayXd conflictingUncertainties(conflictingIds.size());
        for (int i = 0; i < conflictingIds.size(); ++i) {
            conflictingUncertainties(i) = scanlinePool.getScanlineById(conflictingIds(i)).uncertainty;
        }

        return {
            .flags = std::move(flags),
            .conflictingIds = conflictingIds,
            .conflictingUncertainties = conflictingUncertainties
        };
    }

    ScanlineConflictsResult ScanlineConflictSolver::rejectCandidateIfEmpiricalIntersection(
        ScanlineIntersectionInfo &&intersection
    ) {
        LOG_INFO(
            "New uncertainty is infinite, but so are the conflicting scanlines uncertainties. ",
            "Intersects other empirical: ", intersection.flags.empiricalIntersection? "True": "False", "."
        );

        const bool reject = intersection.flags.empiricalIntersection;

        return {
            .shouldReject = reject,
            .conflictingScanlines = reject? std::move(intersection.conflictingIds) : Eigen::ArrayXi(),
        };
    }

    ScanlineConflictsResult ScanlineConflictSolver::rejectCandidate(
        const VerticalScanlineCandidate &candidate, const ScanlineIntersectionInfo &intersection
    ) {
        Eigen::ArrayXi betterScanlineIds = Eigen::ArrayXi::Zero(intersection.conflictingIds.size());
        LOG_INFO("New uncertainty is higher than conflicting scanlines uncertainties. Rejecting current scanline");

        int j = 0;
        for (int i = 0; i < intersection.conflictingIds.size(); ++i) {
            if (candidate.scanline.uncertainty >= intersection.conflictingUncertainties(i)) {
                betterScanlineIds(j++) = intersection.conflictingIds(i);
            }
        }
        betterScanlineIds.conservativeResize(j);

        return {
            .shouldReject = true,
            .conflictingScanlines = std::move(betterScanlineIds)
        };
    }

    ScanlineConflictsResult ScanlineConflictSolver::rejectConflicting(ScanlineIntersectionInfo &&intersection) {
        LOG_INFO("New uncertainty is lower than conflicting scanlines uncertainties. Rejecting conflicting scanlines");

        return {
            .shouldReject = false,
            .conflictingScanlines = std::move(intersection.conflictingIds)
        };
    }

    bool ScanlineConflictSolver::simpleShouldKeep(
        VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        const auto intersectionInfo = computeIntersectionFlags(scanlinePool, candidate);
        const bool conflicting = intersectionInfo.empiricalIntersection;

        if (conflicting) {
            scanlinePool.invalidateByHash(candidate.scanline.hough.cell.hash);
        }

        return !conflicting;
    }
} // accurate_ri
