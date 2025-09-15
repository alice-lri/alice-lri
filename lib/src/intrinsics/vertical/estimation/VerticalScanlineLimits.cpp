#include "VerticalScanlineLimits.h"
#include "point/PointArray.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

namespace alice_lri {

    VerticalBounds VerticalScanlineLimits::computeErrorBounds(const PointArray &points, const double offset) {
        PROFILE_SCOPE("VerticalScanlineLimits::computeErrorBounds");
        const double coordsEps = points.getCoordsEps();
        const auto &zs = points.getZ();
        const auto &rangesXy = points.getRangesXy();
        const auto &ranges = points.getRanges();

        const auto &rangeXySquared = rangesXy.square();
        const auto &rangeSquared = ranges.square();

        const double rangesBound = coordsEps * std::sqrt(3);
        const double rangesXyBound = coordsEps * std::sqrt(2);

        VerticalBounds result;

        const auto phisBoundNumerator = rangesXyBound * zs.cwiseAbs() + coordsEps * rangesXy;
        const auto phisBoundDenominator = rangeXySquared - rangesXyBound * rangesXy;
        const auto correctionBoundNumerator = std::abs(offset) * rangesBound;
        const auto correctionBoundDenominator = rangeSquared - rangesBound * ranges;

        result.phis = phisBoundNumerator / phisBoundDenominator;
        result.correction = correctionBoundNumerator / correctionBoundDenominator;
        result.final = result.phis + result.correction;

        return result;
    }

    ScanlineLimits VerticalScanlineLimits::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const double offset, const double angle,
        const VerticalMargin &margin
    ) {
        PROFILE_SCOPE("VerticalScanlineLimits::computeScanlineLimits");
        const auto &inv = points.getInvRanges();
        const auto &phi = points.getPhis();

        const auto sinUpper = ((offset + margin.offset) * inv.array()).min(1).max(-1).asin();
        const auto sinLower = ((offset - margin.offset) * inv.array()).min(1).max(-1).asin();

        const Eigen::ArrayXd upper = angle + sinUpper + margin.angle + errorBounds.array();
        const Eigen::ArrayXd lower = angle + sinLower - margin.angle - errorBounds.array();

        Eigen::ArrayX<bool> mask = (lower <= phi) && (phi <= upper);
        Eigen::ArrayXi indices = Utils::eigenMaskToIndices(mask);

        return { std::move(indices), std::move(mask) };
    }
}