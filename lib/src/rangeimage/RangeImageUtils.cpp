#include "RangeImageUtils.h"
#include "accurate_ri/public_structs.hpp"
#include <Eigen/Core>

#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    ) {
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

                const double phiCorrection = std::asin(vOffset / ranges(pointIdx));
                const double phiDiff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

                if (phiDiff < minPhiDiff) {
                    minPhiDiff = phiDiff;
                    bestLaserIdx = laserIdx;
                }
            }

            minIndices(pointIdx) = bestLaserIdx;
            const double hOffset = intrinsics.horizontal.scanlines[bestLaserIdx].offset;
            correctedThetas(pointIdx) = thetas(pointIdx) - std::asin(hOffset / rangesXy(pointIdx));
            correctedThetas(pointIdx) = correctedThetas(pointIdx) < 0 ? 2 * M_PI + correctedThetas(pointIdx) : correctedThetas(pointIdx);
        }

        const int32_t maxHorizontalResolution = std::ranges::max_element(
            intrinsics.horizontal.scanlines, {}, [](const auto &scanline) {
                return scanline.resolution;
            }
        )->resolution;

        std::vector<double> pixels(maxHorizontalResolution * intrinsics.vertical.scanlinesCount, 0);

        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            const int32_t row = intrinsics.vertical.scanlinesCount - minIndices(pointIdx) - 1;
            // TODO handle different resolutions per scanline
            int32_t col = static_cast<int32_t>(std::round(
                correctedThetas(pointIdx) / (2 * M_PI) * maxHorizontalResolution
            ));
            col = col < 0 ? maxHorizontalResolution - col : col;
            col = col >= maxHorizontalResolution ? col - maxHorizontalResolution : col;

            pixels[row * maxHorizontalResolution + col] = ranges(pointIdx);
        }

        return RangeImage{
            .width = maxHorizontalResolution,
            // TODO avoid cast
            .height = static_cast<int32_t>(intrinsics.vertical.scanlinesCount),
            .pixels = std::move(pixels)
        };
    }
}
