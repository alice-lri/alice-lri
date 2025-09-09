#include "VerticalHeuristicsEstimator.h"
#include <ranges>
#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/estimation/VerticalHeuristicsStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"
#include "utils/logger/Logger.h"

namespace accurate_ri {
    std::optional<ScanlineEstimationResult> VerticalHeuristicsEstimator::estimate(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
    ) {
        LOG_WARN("Heuristic fitting");
        const HeuristicScanline scanline = computeHeuristicScanline(points, scanlinePool, scanlineLimits);
        const OffsetAngleMargin margin = computeHeuristicMargin(scanlinePool, scanline);
        const VerticalBounds bounds = VerticalScanlineLimits::computeErrorBounds(points, scanline.offset.value);

        const OffsetAngle heuristicOffsetAngle = {scanline.offset.value, scanline.angle.value};
        ScanlineLimits heuristicLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, bounds.final, heuristicOffsetAngle, margin
        );

        if (heuristicLimits.indices.size() == 0) {
            return std::nullopt;
        }

        return ScanlineEstimationResult{
            .heuristic = true,
            .uncertainty = std::numeric_limits<double>::infinity(),
            .values = heuristicOffsetAngle,
            .ci = OffsetAngleMargin{
                .offset = scanline.offset.ci,
                .angle = scanline.angle.ci
            },
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
                const double scanlinePhi = std::asin(scanline.values.offset * invRangesMean) + scanline.values.angle;

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
            const double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;

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
            meanOffset += scanline.values.offset / static_cast<double>(supportScanlineIds.size());
        }

        return meanOffset;
    }

    ValueConfInterval VerticalHeuristicsEstimator::computeHeuristicAngle(
        const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const ValueConfInterval &offset
    ) {
        const double heuristicAngle = (phis - (offset.value * invRanges).asin()).mean();
        const RealMargin angleMarginTmp = {
            .lower = (phis - (offset.ci.lower * invRanges).asin()).mean(),
            .upper = (phis - (offset.ci.upper * invRanges).asin()).mean()
        };

        const RealMargin heuristicAngleCi = {
            .lower = std::min(angleMarginTmp.lower, angleMarginTmp.upper),
            .upper = std::max(angleMarginTmp.lower, angleMarginTmp.upper)
        };

        return ValueConfInterval{
            .value = heuristicAngle,
            .ci = heuristicAngleCi
        };
    }

    OffsetAngleMargin VerticalHeuristicsEstimator::computeHeuristicMargin(
        const VerticalScanlinePool &scanlinePool, const HeuristicScanline &scanline
    ) {
        double offsetMargin = scanline.offset.ci.diff() / 2;
        double angleMargin = scanline.angle.ci.diff() / 2;
        const OffsetAngleMargin houghMargin = scanlinePool.getHoughMargin();

        offsetMargin = std::max(offsetMargin, houghMargin.offset.upper);
        angleMargin = std::max(angleMargin, houghMargin.angle.upper);

        return OffsetAngleMargin {offsetMargin, offsetMargin, angleMargin, angleMargin};
    }
} // accurate_ri
