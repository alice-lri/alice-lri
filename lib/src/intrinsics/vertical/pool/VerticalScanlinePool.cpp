#include "VerticalScanlinePool.h"
#include <optional>
#include <ranges>

#include "Constants.h"
#include "utils/Logger.h"

namespace accurate_ri {
    VerticalScanlinePool::VerticalScanlinePool(
        const double offsetMin, const double offsetMax, const double offsetStep, const double angleMin,
        const double angleMax, const double angleStep
    ) : hough(offsetMin, offsetMax, offsetStep, angleMin, angleMax, angleStep) {}

    void VerticalScanlinePool::performPrecomputations(const PointArray &points) {
        hough.computeAccumulator(points);
        pointsScanlinesIds = Eigen::ArrayXi::Ones(points.size()) * -1;
        unassignedPoints = points.size();
        //hough.debugThing();
    }

    std::optional<HoughScanlineEstimation> VerticalScanlinePool::performHoughEstimation() {
        double averageOffset = 0;
        for (const ScanlineInfo &info: scanlineInfoMap | std::views::values) {
            averageOffset += info.values.offset / static_cast<double>(scanlineInfoMap.size());
        }

        const std::optional<HoughCell> &houghMaxOpt = hough.findMaximum(averageOffset);
        hough.debugPrintCell();

        if (!houghMaxOpt) {
            return std::nullopt;
        }

        return HoughScanlineEstimation{
            .cell = *houghMaxOpt,
            .margin = getHoughMargin()
        };
    }

    void VerticalScanlinePool::assignScanline(ScanlineInfo&& scanline, const Eigen::ArrayXi& pointsIndices) {
        pointsScanlinesIds(pointsIndices) = scanline.id;
        unassignedPoints -= pointsIndices.size();

        scanlineInfoMap.emplace(scanline.id, std::move(scanline));
    }

    std::optional<ScanlineInfo> VerticalScanlinePool::removeScanline(const PointArray &points, const uint32_t scanlineId) {
        const auto node = scanlineInfoMap.extract(scanlineId);
        if (!node) {
            return std::nullopt;
        }

        const ScanlineInfo &scanline = node.mapped();
        const uint64_t hash = scanline.houghHash;
        const double votes = scanline.houghVotes;

        LOG_INFO("Removing scanline ", scanlineId, " with hash ", hash, " and votes ", votes);
        hough.restoreVotes(hash, votes);

        if (scanline.uncertainty < Constant::FULL_CERTAINTY_THRESHOLD) {
            std::vector<int32_t> indicesVector;

            for (int i = 0; i < pointsScanlinesIds.size(); ++i) {
                if (pointsScanlinesIds(i) == scanlineId) {
                    indicesVector.emplace_back(i);
                }
            }

            const Eigen::ArrayXi indices = Eigen::Map<Eigen::ArrayXi>(indicesVector.data(), indicesVector.size());
            hough.restorePoints(points, indices);
        }
        
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

        return FullScanlines {
            .scanlines = std::move(sortedScanlines),
            .pointsScanlinesIds = std::vector(pointsScanlinesIds.data(), pointsScanlinesIds.data() + pointsScanlinesIds.size()),
        };
    }

    OffsetAngleMargin VerticalScanlinePool::getHoughMargin() {
        return OffsetAngleMargin {
            {hough.getXStep(), hough.getXStep()},
            {hough.getYStep(), hough.getYStep()}
        };
    }

    std::vector<ScanlineInfo> VerticalScanlinePool::getUnsortedScanlinesCopy() {
        std::vector<ScanlineInfo> scanlines;
        for (ScanlineInfo &scanlineInfo: scanlineInfoMap | std::views::values) {
            scanlines.emplace_back(scanlineInfo);
        }

        return scanlines;
    }
} // accurate_ri
