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

    inline void VerticalScanlinePool::invalidateHash(const uint64_t hash) {
        hough.eraseByHash(hash);
    }

    inline bool VerticalScanlinePool::anyUnassigned() const {
        return unassignedPoints > 0;
    }
} // accurate_ri
