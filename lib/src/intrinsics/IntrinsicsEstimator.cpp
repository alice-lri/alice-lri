#include "IntrinsicsEstimator.h"

namespace alice_lri {

    Intrinsics IntrinsicsEstimator::estimate(const PointArray &points) {
        const VerticalIntrinsicsEstimation vertical = VerticalIntrinsicsEstimator::estimate(points);
        const HorizontalIntrinsicsEstimation horizontal = HorizontalIntrinsicsEstimator::estimate(points, vertical);

        const int32_t scanlinesCount = static_cast<int32_t>(vertical.scanlinesAssignations.scanlines.size());
        Intrinsics intrinsics(scanlinesCount);

        for (int i = 0; i < scanlinesCount; ++i) {
            const auto& verticalScanline = vertical.scanlinesAssignations.scanlines[i];
            const auto& horizontalScanline = horizontal.scanlines[i];

            intrinsics.scanlines[i] = makeScanline(verticalScanline, horizontalScanline);
        }

        return intrinsics;
    }

    IntrinsicsDetailed IntrinsicsEstimator::debugEstimate(const PointArray &points) {
        const VerticalIntrinsicsEstimation vertical = VerticalIntrinsicsEstimator::estimate(points);
        const HorizontalIntrinsicsEstimation horizontal = HorizontalIntrinsicsEstimator::estimate(points, vertical);

        const int32_t scanlinesCount = static_cast<int32_t>(vertical.scanlinesAssignations.scanlines.size());
        IntrinsicsDetailed intrinsics(scanlinesCount, vertical.iterations, vertical.unassignedPoints,
            vertical.pointsCount, vertical.endReason);

        for (int i = 0; i < scanlinesCount; ++i) {
            const auto& verticalScanline = vertical.scanlinesAssignations.scanlines[i];
            const auto& horizontalScanline = horizontal.scanlines[i];

            intrinsics.scanlines[i] = makeDebugScanline(verticalScanline, horizontalScanline);
        }

        return intrinsics;
    }

    Scanline IntrinsicsEstimator::makeScanline(
        const VerticalScanline &vertical, const HorizontalScanline &horizontal
    ) {
        return Scanline {
            .verticalOffset = vertical.offset.value,
            .verticalAngle = vertical.angle.value,
            .horizontalOffset = horizontal.offset,
            .azimuthalOffset = horizontal.thetaOffset,
            .resolution = horizontal.resolution
        };
    }

    ScanlineDetailed IntrinsicsEstimator::makeDebugScanline(
        const VerticalScanline &vertical, const HorizontalScanline &horizontal
    ) {
        return ScanlineDetailed {
            .verticalOffset = vertical.offset,
            .verticalAngle = vertical.angle,
            .horizontalOffset = horizontal.offset,
            .azimuthalOffset = horizontal.thetaOffset,
            .resolution = horizontal.resolution,
            .uncertainty = vertical.uncertainty,
            .houghVotes = vertical.hough.cell.votes,
            .houghHash = vertical.hough.cell.hash,
            .pointsCount = vertical.pointsCount,
            .theoreticalAngleBounds = vertical.theoreticalAngleBounds,
            .verticalHeuristic = vertical.uncertainty == std::numeric_limits<double>::infinity(),
            .horizontalHeuristic = horizontal.heuristic
        };
    }
}
