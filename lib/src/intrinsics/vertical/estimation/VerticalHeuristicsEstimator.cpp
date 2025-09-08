#include "VerticalHeuristicsEstimator.h"

#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"

namespace accurate_ri {
    std::optional<ScanlineEstimationResult> VerticalHeuristicsEstimator::performHeuristicFit(
        const PointArray &points, const VerticalScanlinePool &scanlinePool, const ScanlineLimits &scanlineLimits
    ) {
        LOG_WARN("Heuristic fitting");
        const Eigen::ArrayXd &invRanges = points.getInvRanges()(scanlineLimits.indices);
        const Eigen::ArrayXd &phis = points.getPhis()(scanlineLimits.indices);

        const double invRangesMean = invRanges.mean();
        const double phisMean = phis.mean();

        HeuristicScanline heuristic = computeHeuristicScanline(scanlinePool, invRangesMean, phisMean);
        heuristic.offsetCi.clampBoth(-points.getMinRange(), points.getMinRange());

        const double heuristicAngle = (phis - (heuristic.offset * invRanges).asin()).mean();

        const RealMargin &angleMarginTmp = {
            .lower = (phis - (heuristic.offsetCi.lower * invRanges).asin()).mean(),
            .upper = (phis - (heuristic.offsetCi.upper * invRanges).asin()).mean()
        };

        const RealMargin &heuristicAngleCi = {
            .lower = std::min(angleMarginTmp.lower, angleMarginTmp.upper),
            .upper = std::max(angleMarginTmp.lower, angleMarginTmp.upper)
        };

        LOG_INFO(
            "Offset confidence interval: ", heuristic.offsetCi, ", Angle confidence interval: ",
            heuristicAngleCi, ", Offset: ", heuristic.offset, ", Angle: ", heuristicAngle
        );

        double offsetMargin = heuristic.offsetCi.diff() / 2;
        double angleMargin = heuristicAngleCi.diff() / 2;
        const OffsetAngleMargin houghMargin = scanlinePool.getHoughMargin();

        // Numerical stability
        offsetMargin = std::max(offsetMargin, houghMargin.offset.upper);
        angleMargin = std::max(angleMargin, houghMargin.angle.upper);

        const VerticalBounds heuristicBounds = VerticalScanlineLimits::computeErrorBounds(points, heuristic.offset);
        OffsetAngle heuristicOffsetAngle = {heuristic.offset, heuristicAngle};

        const OffsetAngleMargin heuristicMargin = {offsetMargin, offsetMargin, angleMargin, angleMargin};
        ScanlineLimits heuristicLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, heuristicBounds.final, heuristicOffsetAngle, heuristicMargin
        );
        LOG_INFO("Offset: ", heuristicOffsetAngle.offset, ", Angle: ", heuristicOffsetAngle.angle);

        if (heuristicLimits.indices.size() == 0) {
            return std::nullopt;
        }

        return ScanlineEstimationResult{
            .heuristic = true,
            .uncertainty = std::numeric_limits<double>::infinity(),
            .values = heuristicOffsetAngle,
            .ci = OffsetAngleMargin{
                .offset = heuristic.offsetCi,
                .angle = heuristicAngleCi
            },
            .limits = std::move(heuristicLimits)
        };
    }

    HeuristicScanline VerticalHeuristicsEstimator::computeHeuristicScanline(
        const VerticalScanlinePool &scanlinePool, const double invRangesMean, const double phisMean
    ) {
        std::optional<uint32_t> closestScanlineIdTop = std::nullopt;
        std::optional<uint32_t> closestScanlineIdBottom = std::nullopt;
        double closestScanlineTopDistance = std::numeric_limits<double>::infinity();
        double closestScanlineBottomDistance = std::numeric_limits<double>::infinity();

        scanlinePool.forEachScanline(
            [&](const ScanlineInfo &scanline) {
                uint32_t scanlineId = scanline.id;

                const double scanlinePhi = std::asin(scanline.values.offset * invRangesMean) + scanline.values.angle;
                if (scanlinePhi > phisMean) {
                    const double distance = scanlinePhi - phisMean;
                    if (distance < closestScanlineTopDistance) {
                        closestScanlineIdTop = scanlineId;
                        closestScanlineTopDistance = distance;
                    }
                }
                if (scanlinePhi < phisMean) {
                    const double distance = phisMean - scanlinePhi;
                    if (distance < closestScanlineBottomDistance) {
                        closestScanlineIdBottom = scanlineId;
                        closestScanlineBottomDistance = distance;
                    }
                }
            }
        );

        std::vector<uint32_t> validScanlineIds;
        if (closestScanlineIdTop) {
            validScanlineIds.emplace_back(*closestScanlineIdTop);
        }
        if (closestScanlineIdBottom) {
            validScanlineIds.emplace_back(*closestScanlineIdBottom);
        }

        assert(!validScanlineIds.empty() && "No valid scanlines found");

        std::vector<double> offsetDiffs;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;
            offsetDiffs.emplace_back(offsetDiff);
        }

        const double maxOffsetDiff = std::ranges::max(offsetDiffs);
        double meanOffset = 0;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlinePool.getScanlineById(scanlineId);
            meanOffset += scanline.values.offset / static_cast<double>(validScanlineIds.size());
        }

        return HeuristicScanline{
            .offset = meanOffset,
            .offsetCi = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2}
        };
    }
} // accurate_ri