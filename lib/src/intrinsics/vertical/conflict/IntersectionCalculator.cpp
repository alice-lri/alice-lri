#include "IntersectionCalculator.h"

namespace accurate_ri {

    ScanlineIntersectionFlags IntersectionCalculator::computeIntersectionFlags(
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

    ScanlineIntersectionInfo IntersectionCalculator::computeIntersectionInfo(
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

    std::vector<bool> IntersectionCalculator::computeEmpiricalIntersections(
        const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
    ) {
        std::vector result(candidate.scanline.id + 1, false);
        const Eigen::ArrayXi &conflictingScanlinesIdsVerbose = scanlinePool.getScanlinesIds(candidate.limits.indices);

        for (const int32_t conflictingId: conflictingScanlinesIdsVerbose) {
            if (conflictingId >= 0) {
                result[conflictingId] = true;
            }
        }

        return result;
    }

    std::vector<bool> IntersectionCalculator::computeTheoreticalIntersections(
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
} // accurate_ri