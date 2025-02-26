#include "VerticalScanlinePool.h"
#include <optional>
#include <ranges>

namespace accurate_ri {
    VerticalScanlinePool::VerticalScanlinePool(
        const double offsetMin, double offsetMax, double offsetStep, double angleMin, double angleMax, double angleStep
    ) : hough(offsetMin, offsetMax, offsetStep, angleMin, angleMax, angleStep) {}

    void VerticalScanlinePool::performPrecomputations(const PointArray &points) {
        hough.computeAccumulator(points);
        pointsScanlinesIds = Eigen::ArrayXi::Ones(points.size()) * -1;
        unassignedPoints = points.size();
    }

    std::optional<HoughScanlineEstimation> VerticalScanlinePool::performHoughEstimation() {
        double averageOffset = 0;
        for (const ScanlineInfo &info: scanlineInfoMap | std::views::values) {
            averageOffset += info.values.offset / static_cast<double>(scanlineInfoMap.size());
        }

        const std::optional<HoughCell> &houghMaxOpt = hough.findMaximum(averageOffset);

        if (!houghMaxOpt) {
            return std::nullopt;
        }

        const HoughCell &houghMax = *houghMaxOpt;

        const OffsetAngleMargin &margin = {
            {hough.getXStep(), hough.getXStep()},
            {hough.getYStep(), hough.getYStep()}
        };

        return HoughScanlineEstimation{
            .cell = std::move(houghMax),
            .margin = std::move(margin)
        };
    }

    void VerticalScanlinePool::assignScanline(ScanlineInfo&& scanline, const Eigen::ArrayXi& pointsIndices) {
        pointsScanlinesIds(pointsIndices) = scanline.id;
        unassignedPoints -= pointsIndices.size();

        scanlineInfoMap.emplace(scanline.id, std::move(scanline));
    }

    std::optional<ScanlineInfo> VerticalScanlinePool::removeScanline(const uint32_t scanlineId) {
        const auto node = scanlineInfoMap.extract(scanlineId);
        if (!node) {
            return std::nullopt;
        }

        const ScanlineInfo &scanline = node.mapped();
        const uint64_t conflictingHash = scanline.houghHash;
        const double conflictingVotes = scanline.houghVotes;

        hough.restoreVotes(conflictingHash, conflictingVotes);
        unassignedPoints += scanline.pointsCount;
        pointsScanlinesIds = (pointsScanlinesIds == scanlineId).select(-1, pointsScanlinesIds);

        scanlineInfoMap.erase(scanlineId);

        return scanline;
    }

    FullScanlines VerticalScanlinePool::extractFullSortedScanlines() {
        std::vector<ScanlineInfo> sortedScanlines;
        for (ScanlineInfo &scanlineInfo: scanlineInfoMap | std::views::values) {
            sortedScanlines.emplace_back(std::move(scanlineInfo));
        }
        scanlineInfoMap.clear();

        std::ranges::sort(
            sortedScanlines, [](const ScanlineInfo &a, const ScanlineInfo &b) {
                return a.values.angle < b.values.angle;
            }
        );

        std::unordered_map<uint32_t, uint32_t> oldIdsToNewIdsMap;
        for (uint32_t i = 0; i < sortedScanlines.size(); ++i) {
            oldIdsToNewIdsMap.emplace(sortedScanlines[i].id, i);
        }

        for (auto &scanlineInfo: sortedScanlines) {
            scanlineInfo.id = oldIdsToNewIdsMap[scanlineInfo.id];
            std::ranges::transform(
                scanlineInfo.dependencies, scanlineInfo.dependencies.begin(), [&oldIdsToNewIdsMap](const uint32_t id) {
                    return oldIdsToNewIdsMap[id];
                }
            );
        }

        std::ranges::transform(
            pointsScanlinesIds, pointsScanlinesIds.begin(), [&oldIdsToNewIdsMap](const int32_t id) {
                return (id >= 0) ? oldIdsToNewIdsMap[id] : -1;
            }
        );

        Eigen::ArrayXi movedScanlinesIds = std::move(pointsScanlinesIds);
        pointsScanlinesIds.resize(0);

        return FullScanlines {
            .scanlines = std::move(sortedScanlines),
            .pointsScanlinesIds = std::move(movedScanlinesIds)
        };
    }
} // accurate_ri
