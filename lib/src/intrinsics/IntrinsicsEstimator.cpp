#include "IntrinsicsEstimator.h"

namespace accurate_ri {

    Intrinsics IntrinsicsEstimator::estimate(const PointArray &points) {
        const VerticalIntrinsicsEstimation vertical = verticalIntrinsicsEstimator.estimate(points);
        const HorizontalIntrinsicsEstimation horizontal = horizontalIntrinsicsEstimator.estimate(points, vertical);

        const std::size_t scanlinesCount = vertical.scanlinesAssignations.scanlines.size();
        Intrinsics intrinsics = Intrinsics(scanlinesCount);

        for (int i = 0; i < scanlinesCount; ++i) {
            const auto& verticalScanline = vertical.scanlinesAssignations.scanlines[i];
            const auto& horizontalScanline = horizontal.scanlines[i];

            intrinsics.scanlineAt(i) = makeScanline(verticalScanline, horizontalScanline);
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
} // accurate_ri
