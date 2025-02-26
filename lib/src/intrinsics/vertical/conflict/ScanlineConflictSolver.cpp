#include "ScanlineConflictSolver.h"

#include <queue>

#include "intrinsics/vertical/VerticalStructs.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {

    ScanlineIntersectionInfo ScanlineConflictSolver::computeScanlineIntersectionInfo(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId
    ) {
        const Eigen::ArrayXi &conflictingScanlinesIdsVerbose = pointsScanlinesIds(scanline.limits.indices);
        Eigen::ArrayX<bool> empiricalIntersectionMask = Eigen::ArrayX<bool>::Constant(scanlineId + 1, false);
        bool empiricalIntersection = false;

        for (const int32_t conflictingId: conflictingScanlinesIdsVerbose) {
            if (conflictingId >= 0) {
                empiricalIntersection = true;
                empiricalIntersectionMask[conflictingId] = true;
            }
        }

        Eigen::ArrayX<bool> theoreticalIntersectionMask = Eigen::ArrayX<bool>::Constant(scanlineId + 1, true);
        const std::vector boundsPointers = {&ScanlineAngleBounds::bottom, &ScanlineAngleBounds::top};

        for (const auto thisBound: boundsPointers) {
            const double thisLower = (angleBounds.*thisBound).lower;
            const double thisUpper = (angleBounds.*thisBound).upper;

            for (const auto otherBound: boundsPointers) {
                Eigen::ArrayXd maxTheoreticalSigns = Eigen::ArrayXd::Ones(scanlineId + 1);
                Eigen::ArrayXd minTheoreticalSigns = Eigen::ArrayXd::Ones(scanlineId + 1);

                for (uint32_t otherId = 0; otherId < scanlineId + 1; ++otherId) {
                    if (!scanlineInfoMap.contains(otherId)) {
                        continue;
                    }

                    const double otherLower = (scanlineInfoMap[otherId].theoreticalAngleBounds.*otherBound).lower;
                    const double otherUpper = (scanlineInfoMap[otherId].theoreticalAngleBounds.*otherBound).upper;

                    minTheoreticalSigns[otherId] = Utils::compare(thisLower, otherLower);
                    maxTheoreticalSigns[otherId] = Utils::compare(thisUpper, otherUpper);
                }

                theoreticalIntersectionMask = theoreticalIntersectionMask && (
                                                  (maxTheoreticalSigns * minTheoreticalSigns).array() != 1).array();
            }
        }

        return {
            .empiricalIntersectionMask = std::move(empiricalIntersectionMask),
            .theoreticalIntersectionMask = std::move(theoreticalIntersectionMask),
            .empiricalIntersection = empiricalIntersection,
            .theoreticalIntersection = theoreticalIntersectionMask.any()
        };
    }

    bool ScanlineConflictSolver::performScanlineConflictResolution(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId,
        const HoughCell &houghMax
    ) {
        const ScanlineConflictsResult &conflicts = evaluateScanlineConflicts(
            angleBounds, scanline, scanlineId, houghMax
        );

        if (conflicts.shouldReject) {
            LOG_INFO("Scanline rejected");
            LOG_INFO("");

            hough->eraseByHash(houghMax.hash);

            if (conflicts.conflictingScanlines.size() > 0) {
                HashToConflictValue &hashToConflictValue = hashesToConflictsMap[houghMax.hash];

                hashToConflictValue.conflictingScanlines.insert(
                    conflicts.conflictingScanlines.begin(), conflicts.conflictingScanlines.end()
                );
                hashToConflictValue.votes = houghMax.votes;
            }

            return false;
        }

        std::queue<uint32_t> scanlinesToRemoveQueue;
        std::unordered_set<uint32_t> scanlinesToRemoveSet;

        for (const auto otherId: conflicts.conflictingScanlines) {
            if (scanlinesToRemoveSet.insert(otherId).second) {
                scanlinesToRemoveQueue.push(otherId);
            }
        }

        while (!scanlinesToRemoveQueue.empty()) {
            const uint32_t otherId = scanlinesToRemoveQueue.front();
            scanlinesToRemoveQueue.pop();

            auto dependencyRange = reverseScanlinesDependencyMap.equal_range(otherId);
            for (auto it = dependencyRange.first; it != dependencyRange.second; ++it) {
                if (scanlinesToRemoveSet.insert(it->second).second) {
                    scanlinesToRemoveQueue.push(it->second);
                }
            }
        }

        LOG_INFO("Removing scanlines: ", scanlinesToRemoveSet);

        for (const uint32_t otherId: scanlinesToRemoveSet) {
            const ScanlineInfo &otherScanline = scanlineInfoMap[otherId];
            const uint64_t conflictingHash = otherScanline.houghHash;
            const double conflictingVotes = otherScanline.houghVotes;

            hough->restoreVotes(conflictingHash, conflictingVotes);
            unassignedPoints += otherScanline.pointsCount;
            pointsScanlinesIds = (pointsScanlinesIds == otherId).select(-1, pointsScanlinesIds);

            // TODO careful with this, perhaps select
            scanlineInfoMap.erase(otherId);
            reverseScanlinesDependencyMap.erase(otherId);

            // Remove entries where scanlineId is in the value
            for (auto it = reverseScanlinesDependencyMap.begin(); it != reverseScanlinesDependencyMap.end();) {
                if (it->second == otherId) {
                    it = reverseScanlinesDependencyMap.erase(it);
                } else {
                    ++it;
                }
            }

            for (auto it = hashesToConflictsMap.begin(); it != hashesToConflictsMap.end();) {
                const uint64_t hash = it->first;
                HashToConflictValue &hashToConflictValue = it->second;

                hashToConflictValue.conflictingScanlines.erase(otherId);

                if (hashToConflictValue.conflictingScanlines.empty()) {
                    LOG_INFO("Restored hash: ", hash);

                    hough->restoreVotes(hash, hashToConflictValue.votes);
                    it = hashesToConflictsMap.erase(it);
                } else {
                    ++it;
                }
            }

            // Store the removed line, as conflicting with current one
            hashesToConflictsMap.emplace(
                conflictingHash, HashToConflictValue{
                    .conflictingScanlines = {scanlineId},
                    .votes = conflictingVotes
                }
            );

            LOG_INFO("Added hash ", conflictingHash, " to the map");
        }

        return true;
    }

    // TODO review this method, maybe precompute actually conflicting scanlines and do ifs based on that
    ScanlineConflictsResult ScanlineConflictSolver::evaluateScanlineConflicts(
        const ScanlineAngleBounds &angleBounds, const ScanlineEstimationResult &scanline, const uint32_t scanlineId,
        const HoughCell &houghMax
    ) {
        const ScanlineIntersectionInfo &intersectionInfo = computeScanlineIntersectionInfo(
            angleBounds, scanline, scanlineId
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
            ", Points in scanline: ", scanline.limits.indices.size(), " vs ", houghMax.votes
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
            conflictingScanlinesUncertainties(i) = scanlineInfoMap[conflictingScanlines(i)].uncertainty;
        }

        LOG_INFO(
            "Intersects with scanlines: ", conflictingScanlines,
            ", Current scanline uncertainty: ", scanline.uncertainty,
            ", Conflicting scanlines uncertainties: ", conflictingScanlinesUncertainties
        );

        if (scanline.limits.indices.size() == unassignedPoints) {
            LOG_WARN(
                "Warning: This is the last scanline, so we will accept it if it is "
                "not empirically intersecting with other scanlines"
            );
            // TODO This is causing problems, for example in durlar_4d_single_upper_bound_.._.._datasets_durlar_dataset_DurLAR_DurLAR_20210716_ouster_points_data_0000035800.bin/output.txt
            // TODO maybe recover last scanline flag (evaluate whether this case occurs)
            return {
                .shouldReject = intersectionInfo.empiricalIntersection,
                .conflictingScanlines = std::move(conflictingScanlines)
            };
        }

        if (scanline.uncertainty >= conflictingScanlinesUncertainties.minCoeff()) {
            if (scanline.uncertainty == std::numeric_limits<double>::infinity()) {
                LOG_INFO(
                    "New uncertainty is infinite, but so are the conflicting scanlines uncertainties. Intersects other empirical: ",
                    intersectionInfo.empiricalIntersection? "True": "False", "."
                );

                return {
                    .shouldReject = intersectionInfo.empiricalIntersection,
                    .conflictingScanlines = std::move(conflictingScanlines)
                };
            }

            // TODO thoroughly test this case both in python and C++
            LOG_INFO(
                "New uncertainty is higher than conflicting scanlines uncertainties. Rejecting current scanline"
            );

            Eigen::ArrayXi actuallyConflictingScanlines = Eigen::ArrayXi::Zero(conflictingScanlines.size());

            j = 0;
            for (int i = 0; i < conflictingScanlines.size(); ++i) {
                if (scanline.uncertainty >= conflictingScanlinesUncertainties(i)) {
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
} // accurate_ri