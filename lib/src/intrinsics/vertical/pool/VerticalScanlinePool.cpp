#include "VerticalScanlinePool.h"
#include <optional>
#include <ranges>
#include "utils/Logger.h"

namespace accurate_ri {
    VerticalScanlinePool::VerticalScanlinePool(
        const double offsetMin, const double offsetMax, const double offsetStep, const double angleMin,
        const double angleMax, const double angleStep
    ) : hough(offsetMin, offsetMax, offsetStep, angleMin, angleMax, angleStep) {}

    void VerticalScanlinePool::performPrecomputations(const PointArray &points) {
        hough.computeAccumulator(points);
        pointsScanlinesIds = Eigen::ArrayXi::Ones(static_cast<Eigen::Index>(points.size())) * -1;
        unassignedPoints = static_cast<int64_t>(points.size());
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

        return HoughScanlineEstimation{
            .cell = *houghMaxOpt,
            .margin = getHoughMargin()
        };
    }

    void VerticalScanlinePool::assignScanline(const ScanlineInfo &scanline, const Eigen::ArrayXi& pointsIndices) {
        pointsScanlinesIds(pointsIndices) = static_cast<const int>(scanline.id);
        unassignedPoints -= pointsIndices.size();

        scanlineInfoMap.emplace(scanline.id, scanline);
    }

    std::optional<ScanlineInfo> VerticalScanlinePool::removeScanline(const PointArray &points, const uint32_t scanlineId) {
        const auto node = scanlineInfoMap.extract(scanlineId);
        if (!node) {
            return std::nullopt;
        }

        const Eigen::ArrayXi indices = scanlineIdToPointsIndices(scanlineId);
        const ScanlineInfo &scanline = node.mapped();

        unassignedPoints += static_cast<int64_t>(scanline.pointsCount);
        pointsScanlinesIds = (pointsScanlinesIds == static_cast<const int>(scanlineId)).select(-1, pointsScanlinesIds);

        hough.addVotes(points, indices);
        scanlineInfoMap.erase(scanlineId);

        return scanline;
    }

    Eigen::ArrayXi VerticalScanlinePool::scanlineIdToPointsIndices(const uint32_t scanlineId) const {
        std::vector<int32_t> indicesVector;

        for (int i = 0; i < pointsScanlinesIds.size(); ++i) {
            if (pointsScanlinesIds(i) == scanlineId) {
                indicesVector.emplace_back(i);
            }
        }

        return Eigen::Map<Eigen::ArrayXi>(indicesVector.data(), static_cast<Eigen::Index>(indicesVector.size()));
    }

    FullScanlines VerticalScanlinePool::extractFullSortedScanlines() {
        std::vector<ScanlineInfo> sortedScanlines = computeSortedScanlines();
        updateScanlineIds(sortedScanlines);

        return FullScanlines {
            .scanlines = std::move(sortedScanlines),
            .pointsScanlinesIds = std::vector(pointsScanlinesIds.data(), pointsScanlinesIds.data() + pointsScanlinesIds.size()),
        };
    }

    OffsetAngleMargin VerticalScanlinePool::getHoughMargin() const {
        return OffsetAngleMargin {
            {hough.getXStep(), hough.getXStep()},
            {hough.getYStep(), hough.getYStep()}
        };
    }

    std::vector<ScanlineInfo> VerticalScanlinePool::getUnsortedScanlinesCopy() const {
        std::vector<ScanlineInfo> scanlines;
        for (const ScanlineInfo &scanlineInfo: scanlineInfoMap | std::views::values) {
            scanlines.emplace_back(scanlineInfo);
        }

        return scanlines;
    }

    std::vector<ScanlineInfo> VerticalScanlinePool::computeSortedScanlines() const {
        std::vector<ScanlineInfo> sortedScanlines;
        for (const ScanlineInfo &scanlineInfo: scanlineInfoMap | std::views::values) {
            sortedScanlines.emplace_back(scanlineInfo);
        }

        std::ranges::sort(
            sortedScanlines, [](const ScanlineInfo &a, const ScanlineInfo &b) {
                return a.values.angle < b.values.angle;
            }
        );

        return sortedScanlines;
    }

    void VerticalScanlinePool::updateScanlineIds(std::vector<ScanlineInfo> sortedScanlines) {
        std::unordered_map<uint32_t, uint32_t> oldIdsToNewIdsMap;
        for (uint32_t i = 0; i < sortedScanlines.size(); ++i) {
            oldIdsToNewIdsMap.emplace(sortedScanlines[i].id, i);
        }

        for (auto &scanlineInfo: sortedScanlines) {
            scanlineInfo.id = oldIdsToNewIdsMap[scanlineInfo.id];
        }

        std::ranges::transform(
            pointsScanlinesIds, pointsScanlinesIds.begin(), [&oldIdsToNewIdsMap](const int32_t id) {
                return (id >= 0) ? oldIdsToNewIdsMap[id] : -1;
            }
        );
    }
} // accurate_ri
