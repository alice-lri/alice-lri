#include "ScanlineConflictEvaluator.h"

#include "intrinsics/vertical/helper/VerticalLogging.h"

namespace alice_lri {

    ScanlineConflicts ScanlineConflictEvaluator::evaluateConflicts(
    const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
) {
        auto intersectionFlags = computeIntersectionFlags(scanlinePool, candidate);
        if (!intersectionFlags.anyIntersection()) {
            return {
                .shouldReject = false,
                .conflictingScanlines = std::vector<uint32_t>()
            };
        }

        auto intersectionInfo = computeIntersectionInfo(scanlinePool, std::move(intersectionFlags));
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

    ScanlineIntersectionFlags ScanlineConflictEvaluator::computeIntersectionFlags(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        std::vector<bool> empiricalIntersectionMask = computeEmpiricalIntersections(scanlinePool, candidate);
        const bool empiricalIntersection = std::ranges::any_of(empiricalIntersectionMask, std::identity{});

        std::vector<bool> theoreticalIntersectionMask = computeTheoreticalIntersections(scanlinePool, candidate);
        const bool theoreticalIntersection = std::ranges::any_of(theoreticalIntersectionMask, std::identity{});

        return {
            .empiricalIntersectionMask = std::move(empiricalIntersectionMask),
            .theoreticalIntersectionMask = std::move(theoreticalIntersectionMask),
            .empiricalIntersection = empiricalIntersection,
            .theoreticalIntersection = theoreticalIntersection
        };
    }

    ScanlineIntersectionInfo ScanlineConflictEvaluator::computeIntersectionInfo(
        const VerticalScanlinePool &scanlinePool, ScanlineIntersectionFlags &&flags
    ) {
        std::vector<uint32_t> conflictingIds;
        conflictingIds.reserve(flags.empiricalIntersectionMask.size());

        for (int i = 0; i < flags.empiricalIntersectionMask.size(); ++i) {
            if (flags.anyIntersection(i)) {
                conflictingIds.emplace_back(i);
            }
        }

        std::vector<double> conflictingUncertainties(conflictingIds.size());
        for (int i = 0; i < conflictingIds.size(); ++i) {
            conflictingUncertainties[i] = scanlinePool.getScanlineById(conflictingIds[i]).uncertainty;
        }

        return {
            .flags = std::move(flags),
            .conflictingIds = conflictingIds,
            .conflictingUncertainties = conflictingUncertainties
        };
    }

    std::vector<bool> ScanlineConflictEvaluator::computeEmpiricalIntersections(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        std::vector result(candidate.scanline.id + 1, false);
        const Eigen::ArrayXi conflictingScanlinesIdsVerbose = scanlinePool.getScanlinesIds(candidate.limits.indices);

        for (const int32_t conflictingId: conflictingScanlinesIdsVerbose) {
            if (conflictingId >= 0) {
                result[conflictingId] = true;
            }
        }

        return result;
    }

    std::vector<bool> ScanlineConflictEvaluator::computeTheoreticalIntersections(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        std::vector result(candidate.scanline.id + 1, false);

        const std::vector boundsLinesPointers = {&ScanlineAngleBounds::lowerLine, &ScanlineAngleBounds::upperLine};
        const ScanlineAngleBounds &angleBounds = candidate.scanline.theoreticalAngleBounds;

        scanlinePool.forEachScanline([&](const VerticalScanline &otherScanline) {
            bool allIntersect = true;
            for (const auto thisLinePtr: boundsLinesPointers) {
                const Interval& thisLine = angleBounds.*thisLinePtr;

                for (const auto otherLinePtr: boundsLinesPointers) {
                    const Interval& otherLine = otherScanline.theoreticalAngleBounds.*otherLinePtr;
                    allIntersect = allIntersect && otherLine.anyContained(thisLine);
                }
            }

            result[otherScanline.id] = allIntersect;
        });

        return result;
    }

    ScanlineConflicts ScanlineConflictEvaluator::rejectCandidateIfEmpiricalIntersection(
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

    ScanlineConflicts ScanlineConflictEvaluator::rejectCandidate(
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

    ScanlineConflicts ScanlineConflictEvaluator::rejectConflicting(ScanlineIntersectionInfo &&intersection) {
        LOG_INFO("New uncertainty is lower than conflicting scanlines uncertainties. Rejecting conflicting scanlines");

        return {
            .shouldReject = false,
            .conflictingScanlines = std::move(intersection.conflictingIds)
        };
    }
}