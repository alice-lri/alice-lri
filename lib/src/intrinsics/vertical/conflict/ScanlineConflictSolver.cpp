#include "ScanlineConflictSolver.h"

#include <queue>
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/logger/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {

    bool ScanlineConflictSolver::performScanlineConflictResolution(
        VerticalScanlinePool &scanlinePool, const PointArray &points, const VerticalScanlineCandidate &candidate
    ) {
        const ScanlineConflictsResult &conflicts = evaluateScanlineConflicts(scanlinePool, candidate);

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

    // TODO review this method, maybe precompute actually conflicting scanlines and do ifs based on that
    ScanlineConflictsResult ScanlineConflictSolver::evaluateScanlineConflicts(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        const ScanlineIntersectionInfo &intersectionInfo = computeIntersections(
            scanlinePool, candidate
        );

        if (!intersectionInfo.anyIntersection()) {
            return {
                .shouldReject = false,
                .conflictingScanlines = Eigen::ArrayXi()
            };
        }

        LOG_WARN("Possible problem detected");
        LOG_INFO(
            "Intersects other scanline: ", intersectionInfo.empiricalIntersection? "True": "False",
            ", Intersects theoretically: ", intersectionInfo.theoreticalIntersection,
            ", Fit success: ", "True",
            ", Points in scanline: ", candidate.limits.indices.size(), " vs ", candidate.scanline.hough.cell.votes
        );


        Eigen::ArrayXi conflictingScanlines = Eigen::ArrayXi::Zero(intersectionInfo.empiricalIntersectionMask.size());
        int j = 0;
        for (int i = 0; i < intersectionInfo.empiricalIntersectionMask.size(); ++i) {
            if (intersectionInfo.anyIntersection(i)) {
                conflictingScanlines(j++) = i;
            }
        }
        conflictingScanlines.conservativeResize(j);

        Eigen::ArrayXd conflictingScanlinesUncertainties(conflictingScanlines.size());
        for (int i = 0; i < conflictingScanlines.size(); ++i) {
            conflictingScanlinesUncertainties(i) = scanlinePool.getScanlineById(conflictingScanlines(i)).uncertainty;
        }

        LOG_INFO(
            "Intersects with scanlines: ", conflictingScanlines,
            ", Current scanline uncertainty: ", candidate.scanline.uncertainty,
            ", Conflicting scanlines uncertainties: ", conflictingScanlinesUncertainties
        );

        const double minConflictingUncertainty = conflictingScanlinesUncertainties.minCoeff() - 1e-6;
        if (candidate.scanline.uncertainty >= minConflictingUncertainty) {
            if (minConflictingUncertainty == std::numeric_limits<double>::infinity()) {
                LOG_INFO(
                    "New uncertainty is infinite, but so are the conflicting scanlines uncertainties. Intersects other empirical: ",
                    intersectionInfo.empiricalIntersection? "True": "False", "."
                );

                const bool reject = intersectionInfo.empiricalIntersection;

                return {
                    .shouldReject = reject,
                    .conflictingScanlines = reject? std::move(conflictingScanlines) : Eigen::ArrayXi(),
                };
            }

            LOG_INFO(
                "New uncertainty is higher than conflicting scanlines uncertainties. Rejecting current scanline"
            );

            Eigen::ArrayXi actuallyConflictingScanlines = Eigen::ArrayXi::Zero(conflictingScanlines.size());

            j = 0;
            for (int i = 0; i < conflictingScanlines.size(); ++i) {
                if (candidate.scanline.uncertainty >= conflictingScanlinesUncertainties(i)) {
                    actuallyConflictingScanlines(j++) = conflictingScanlines(i);
                }
            }
            actuallyConflictingScanlines.conservativeResize(j);

            return {
                .shouldReject = true,
                .conflictingScanlines = std::move(actuallyConflictingScanlines)
            };
        }

        LOG_INFO(
            "New uncertainty is lower than conflicting scanlines uncertainties. Rejecting conflicting scanlines"
        );

        return {
            .shouldReject = false,
            .conflictingScanlines = std::move(conflictingScanlines)
        };
    }

    ScanlineIntersectionInfo ScanlineConflictSolver::computeIntersections(
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

    bool ScanlineConflictSolver::simpleShouldKeep(
        VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        const auto intersectionInfo = computeIntersections(scanlinePool, candidate);
        const bool conflicting = intersectionInfo.empiricalIntersection;

        if (conflicting) {
            scanlinePool.invalidateByHash(candidate.scanline.hough.cell.hash);
        }

        return !conflicting;
    }
} // accurate_ri
