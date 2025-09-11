#include "VerticalHeuristicsEstimator.h"
#include <ranges>
#include "intrinsics/vertical/estimation/VerticalScanlineEstimationStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"
#include "utils/logger/Logger.h"

namespace accurate_ri {
    std::optional<VerticalScanlineEstimation> VerticalHeuristicsEstimator::estimate(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
    ) {
        LOG_INFO("Heuristic fitting");
        const HeuristicScanline scanline = computeHeuristicScanline(points, scanlinePool, scanlineLimits);
        const VerticalMargin margin = computeHeuristicMargin(scanlinePool, scanline);
        const VerticalBounds bounds = VerticalScanlineLimits::computeErrorBounds(points, scanline.offset.value);

        ScanlineLimits heuristicLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, bounds.final, scanline.offset.value, scanline.angle.value, margin
        );

        if (heuristicLimits.indices.size() == 0) {
            return std::nullopt;
        }

        return VerticalScanlineEstimation{
            .heuristic = true,
            .uncertainty = std::numeric_limits<double>::infinity(),
            .offset = scanline.offset,
            .angle = scanline.angle,
            .limits = std::move(heuristicLimits)
        };
    }

    HeuristicScanline VerticalHeuristicsEstimator::computeHeuristicScanline(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &limits
    ) {
        const Eigen::ArrayXd &invRanges = points.getInvRanges()(limits.indices);
        const Eigen::ArrayXd &phis = points.getPhis()(limits.indices);

        const double invRangesMean = invRanges.mean();
        const double phisMean = phis.mean();

        ValueConfInterval offset = computeHeuristicOffset(scanlinePool, invRangesMean, phisMean);
        offset.ci.clampBoth(-points.getMinRange(), points.getMinRange());
        const ValueConfInterval angle = computeHeuristicAngle(invRanges, phis, offset);

        LOG_INFO(
            "Offset confidence interval: ", offset.ci, ", Angle confidence interval: ",
            angle.ci, ", Offset: ", offset.value, ", Angle: ", angle.value
        );

        return HeuristicScanline{
            .offset = offset,
            .angle = angle
        };
    }

    ValueConfInterval VerticalHeuristicsEstimator::computeHeuristicOffset(
        const VerticalScanlinePool &scanlinePool, const double invRangesMean, const double phisMean
    ) {
        const std::vector<uint32_t> supportScanlineIds = findSupportScanlines(scanlinePool, invRangesMean, phisMean);
        assert(!supportScanlineIds.empty() && "No valid scanlines found");

        const double maxOffsetDiff = computeMaxOffsetDiff(scanlinePool, supportScanlineIds);
        const double meanOffset = computeMeanOffset(scanlinePool, supportScanlineIds);

        return ValueConfInterval{
            .value = meanOffset,
            .ci = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2}
        };
    }

    std::vector<uint32_t> VerticalHeuristicsEstimator::findSupportScanlines(
        const VerticalScanlinePool &scanlinePool, const double invRangesMean, const double phisMean
    ) {
        HeuristicSupportScanlinePair support;

        scanlinePool.forEachScanline(
            [&](const VerticalScanline &scanline) {
                const uint32_t scanlineId = scanline.id;
                const double scanlinePhi = std::asin(scanline.offset.value * invRangesMean) + scanline.angle.value;

                std::optional<HeuristicSupportScanline>* target = nullptr;
                if (scanlinePhi > phisMean) {
                    target = &support.top;
                } else if (scanlinePhi < phisMean) {
                    target = &support.bottom;
                }

                if (target == nullptr) {
                    return;
                }

                const double distance = std::abs(scanlinePhi - phisMean);
                if (!target->has_value() || distance < target->value().distance) {
                    *target = HeuristicSupportScanline{scanlineId, distance};
                }
            }
        );

        std::vector<uint32_t> validScanlineIds;
        const std::array supports = {support.top, support.bottom};
        for (const auto& it : supports) {
            if (it) {
                validScanlineIds.emplace_back(it->id);
            }
        }

        return validScanlineIds;
    }

    double VerticalHeuristicsEstimator::computeMaxOffsetDiff(
        const VerticalScanlinePool &scanlinePool, const std::vector<uint32_t>& supportScanlineIds
    ) {
        double maxOffsetDiff = 0;

        for (const auto &scanlineId: supportScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            const double offsetDiff = scanline.offset.ci.upper - scanline.offset.ci.lower;

            if (offsetDiff > maxOffsetDiff) {
                maxOffsetDiff = offsetDiff;
            }
        }

        return maxOffsetDiff;
    }

    double VerticalHeuristicsEstimator::computeMeanOffset(
        const VerticalScanlinePool &scanlinePool, const std::vector<uint32_t>& supportScanlineIds
    ) {
        double meanOffset = 0;

        for (const auto &scanlineId: supportScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            meanOffset += scanline.offset.value / static_cast<double>(supportScanlineIds.size());
        }

        return meanOffset;
    }

    ValueConfInterval VerticalHeuristicsEstimator::computeHeuristicAngle(
        const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const ValueConfInterval &offset
    ) {
        const double heuristicAngle = (phis - (offset.value * invRanges).asin()).mean();
        const Interval angleInterval = {
            .lower = (phis - (offset.ci.lower * invRanges).asin()).mean(),
            .upper = (phis - (offset.ci.upper * invRanges).asin()).mean()
        };

        const Interval heuristicAngleCi = {
            .lower = std::min(angleInterval.lower, angleInterval.upper),
            .upper = std::max(angleInterval.lower, angleInterval.upper)
        };

        return ValueConfInterval{
            .value = heuristicAngle,
            .ci = heuristicAngleCi
        };
    }

    VerticalMargin VerticalHeuristicsEstimator::computeHeuristicMargin(
        const VerticalScanlinePool &scanlinePool, const HeuristicScanline &scanline
    ) {
        double offsetMargin = scanline.offset.ci.diff() / 2;
        double angleMargin = scanline.angle.ci.diff() / 2;
        const VerticalMargin houghMargin = scanlinePool.getHoughMargin();

        offsetMargin = std::max(offsetMargin, houghMargin.offset);
        angleMargin = std::max(angleMargin, houghMargin.angle);

        return VerticalMargin {.offset = offsetMargin, .angle = angleMargin};
    }
} // accurate_ri
