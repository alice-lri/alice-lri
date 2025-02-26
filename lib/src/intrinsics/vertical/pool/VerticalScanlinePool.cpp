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
} // accurate_ri
