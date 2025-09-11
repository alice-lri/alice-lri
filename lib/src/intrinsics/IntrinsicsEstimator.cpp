#include "IntrinsicsEstimator.h"

namespace accurate_ri {

    Intrinsics IntrinsicsEstimator::estimate(const PointArray &points) {
        const VerticalIntrinsicsEstimation vertical = VerticalIntrinsicsEstimator::estimate(points);
        const HorizontalIntrinsicsEstimation horizontal = HorizontalIntrinsicsEstimator::estimate(points, vertical);

        const std::size_t scanlinesCount = vertical.scanlinesAssignations.scanlines.size();
        Intrinsics intrinsics = Intrinsics(scanlinesCount);

        for (int i = 0; i < scanlinesCount; ++i) {
            const auto& verticalScanline = vertical.scanlinesAssignations.scanlines[i];
            const auto& horizontalScanline = horizontal.scanlines[i];

            intrinsics.scanlineAt(i) = makeScanline(verticalScanline, horizontalScanline);
        }

        return intrinsics;
    }

    DebugIntrinsics IntrinsicsEstimator::debugEstimate(const PointArray &points) {
        const VerticalIntrinsicsEstimation vertical = VerticalIntrinsicsEstimator::estimate(points);
        const HorizontalIntrinsicsEstimation horizontal = HorizontalIntrinsicsEstimator::estimate(points, vertical);

        const std::size_t scanlinesCount = vertical.scanlinesAssignations.scanlines.size();
        DebugIntrinsics intrinsics = DebugIntrinsics(scanlinesCount, vertical.iterations, vertical.unassignedPoints,
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

    DebugScanline IntrinsicsEstimator::makeDebugScanline(
        const VerticalScanline &vertical, const HorizontalScanline &horizontal
    ) {
        return DebugScanline {
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
} // accurate_ri
