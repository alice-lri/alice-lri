#include "RangeImageUtils.h"
#include <numbers>
#include <Eigen/Core>
#include "accurate_ri/public_structs.hpp"
#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    ) { // TODO split method
        PROFILE_SCOPE("RangeImageUtils::computeRangeImage");
        const Eigen::ArrayXd rangesXySquared = x.square() + y.square();
        const Eigen::ArrayXd ranges = (rangesXySquared + z.square()).sqrt();
        const Eigen::ArrayXd rangesXy = rangesXySquared.sqrt();
        const Eigen::ArrayXd phis = (z / ranges).asin();
        const Eigen::ArrayXd thetas = y.binaryExpr(
            x, [](const double yi, const double xi) {
                return std::atan2(yi, xi) + M_PI;
            }
        );

        Eigen::ArrayXi minIndices(phis.size());
        Eigen::ArrayXd correctedThetas(phis.size());
        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            double minPhiDiff = std::numeric_limits<double>::max();
            int32_t bestLaserIdx = -1;

            for (int32_t laserIdx = 0; laserIdx < intrinsics.vertical.scanlinesCount; ++laserIdx) {
                // TODO full scanlines here is craaaazy
                const double vOffset = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.offset;
                const double vAngle = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.angle;

                const double phiCorrection = vOffset / ranges(pointIdx);
                const double phiDiff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

                if (phiDiff < minPhiDiff) {
                    minPhiDiff = phiDiff;
                    bestLaserIdx = laserIdx;
                }
            }

            minIndices(pointIdx) = bestLaserIdx;
            const double hOffset = intrinsics.horizontal.scanlines[bestLaserIdx].offset;
            const double thetaOffset = intrinsics.horizontal.scanlines[bestLaserIdx].thetaOffset;
            correctedThetas(pointIdx) = thetas(pointIdx) - hOffset / rangesXy(pointIdx) - thetaOffset;
            correctedThetas(pointIdx) = correctedThetas(pointIdx) < 0 ? 2 * M_PI + correctedThetas(pointIdx) : correctedThetas(pointIdx); // TODO this should wrap around on both sides
            correctedThetas(pointIdx) = correctedThetas(pointIdx) > 2 * M_PI ? correctedThetas(pointIdx) - 2 * M_PI : correctedThetas(pointIdx);
        }

        const int32_t maxHorizontalResolution = std::ranges::max_element(
            intrinsics.horizontal.scanlines, {}, [](const auto &scanline) {
                return scanline.resolution;
            }
        )->resolution;

        RangeImage rangeImage(maxHorizontalResolution, intrinsics.vertical.scanlinesCount, 0);

        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            const uint32_t row = intrinsics.vertical.scanlinesCount - minIndices(pointIdx) - 1;
            // TODO handle different resolutions per scanline
            const double thetaStep = 2 * std::numbers::pi / static_cast<double>(maxHorizontalResolution);
            int32_t col = static_cast<int32_t>(std::round(correctedThetas(pointIdx) / thetaStep));

            col = col < 0 ? maxHorizontalResolution - col : col;
            col = col >= maxHorizontalResolution ? col - maxHorizontalResolution : col;

            if (rangeImage(row, col) != 0) {
                LOG_WARN("Overwriting pixel at (", row, ", ", col, ") with range ", ranges(pointIdx),
                         " (previously: ", rangeImage(row, col), ")");
            }

            rangeImage(row, col) = ranges(pointIdx);
        }

        return rangeImage;
    }

    PointCloud::Double unProjectRangeImage(const IntrinsicsResult &intrinsics, const RangeImage &image) {
        std::vector<double> xs, ys, zs;

        for (int32_t row = 0; row < image.height; ++row) {
            const uint32_t scanlineIdx = image.height - row - 1;
            const auto verticalValues = intrinsics.vertical.fullScanlines.scanlines[scanlineIdx].values;
            const auto horizontalValues = intrinsics.horizontal.scanlines[scanlineIdx];

            for (int32_t col = 0; col < image.width; ++col) {
                const double range = image(row, col);

                if (range <= 0) {
                    continue;
                }

                const double originalPhi = verticalValues.angle + verticalValues.offset / range;
                const double rangeXy = range * std::cos(originalPhi);
                const double thetaOffset = horizontalValues.thetaOffset;
                double originalTheta = col * 2 * M_PI / horizontalValues.resolution - M_PI; // TODO handle different resolutions per scanline
                originalTheta += horizontalValues.offset / rangeXy + thetaOffset; // TODO wrap around on both sides

                xs.emplace_back(rangeXy * std::cos(originalTheta));
                ys.emplace_back(rangeXy * std::sin(originalTheta));
                zs.emplace_back(range * std::sin(originalPhi));
            }
        }

        return PointCloud::Double(std::move(xs), std::move(ys), std::move(zs));
    }
}
