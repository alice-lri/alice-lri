#include "VerticalScanlineLimits.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "point/PointArray.h"
#include "utils/Timer.h"

namespace accurate_ri {
    // TODO precompute (whatever is possible) on PointArray
    VerticalBounds VerticalScanlineLimits::computeErrorBounds(const PointArray &points, const double offset) {
        PROFILE_SCOPE("VerticalScanlineLimits::computeErrorBounds");
        double coordsEps = points.getCoordsEps();
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

    // TODO this could be a generic line thing. Take a look at conceptually section in scanlimie limits notebook
    ScanlineLimits VerticalScanlineLimits::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const OffsetAngleMargin &margin, const double invRangesShift // TODO unused
    ) {
        PROFILE_SCOPE("VerticalScanlineLimits::computeScanlineLimits");
        const auto &inv = points.getInvRanges();
        const auto &phi = points.getPhis();
        const double offset = scanlineAttributes.offset;
        const double angle = scanlineAttributes.angle;

        const auto sinUpper = ((offset + margin.offset.upper) * inv.array()).min(1).max(-1).asin();
        const auto sinLower = ((offset - margin.offset.lower) * inv.array()).min(1).max(-1).asin();

        Eigen::ArrayXd upper = angle + sinUpper + margin.angle.upper + errorBounds.array();
        Eigen::ArrayXd lower = angle + sinLower - margin.angle.lower - errorBounds.array();
        Eigen::ArrayX<bool> mask = (lower <= phi) && (phi <= upper);
        Eigen::ArrayXi idx(mask.count());

        int k = 0;
        for (Eigen::Index i = 0; i < mask.size(); ++i) {
            if (mask[i]) {
                idx[k++] = static_cast<int>(i);
            }
        }

        return { std::move(idx), std::move(mask), std::move(lower), std::move(upper) };
    }
} // accurate_ri