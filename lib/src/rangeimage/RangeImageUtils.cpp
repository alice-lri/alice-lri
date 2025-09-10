#include "RangeImageUtils.h"
#include <numbers>
#include <numeric>
#include <Eigen/Core>
#include "accurate_ri/public_structs.hpp"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"
#include "Constants.h"

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const Intrinsics &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    ) { // TODO split method
        PROFILE_SCOPE("RangeImageUtils::computeRangeImage");
        const Eigen::ArrayXd rangesXySquared = x.square() + y.square();
        const Eigen::ArrayXd ranges = (rangesXySquared + z.square()).sqrt();
        const Eigen::ArrayXd rangesXy = rangesXySquared.sqrt();
        const Eigen::ArrayXd phis = (z / ranges).asin();
        const Eigen::ArrayXd thetas = y.binaryExpr(
            x, [](const double yi, const double xi) {
                return std::atan2(yi, xi) + std::numbers::pi;
            }
        );

        Eigen::ArrayXi minIndices(phis.size());
        Eigen::ArrayXd correctedThetas(phis.size());
        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            double minPhiDiff = std::numeric_limits<double>::max();
            int32_t bestLaserIdx = -1;

            for (int32_t laserIdx = 0; laserIdx < intrinsics.scanlinesCount(); ++laserIdx) {
                const double vOffset = intrinsics.scanlineAt(laserIdx).verticalOffset;
                const double vAngle = intrinsics.scanlineAt(laserIdx).verticalAngle;

                const double phiCorrection = vOffset / ranges(pointIdx);
                const double phiDiff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

                if (phiDiff < minPhiDiff) {
                    minPhiDiff = phiDiff;
                    bestLaserIdx = laserIdx;
                }
            }

            minIndices(pointIdx) = bestLaserIdx;
            const double hOffset = intrinsics.scanlineAt(bestLaserIdx).horizontalOffset;
            const double thetaOffset = intrinsics.scanlineAt(bestLaserIdx).azimuthalOffset;
            correctedThetas(pointIdx) = thetas(pointIdx) - hOffset / rangesXy(pointIdx) - thetaOffset;
        }

        Utils::positiveFmodInplace(correctedThetas, Constant::TWO_PI);

        int32_t lcmHorizontalResolution = 1;
        for (int i = 0; i < intrinsics.scanlinesCount(); ++i) {
            const auto &scanline = intrinsics.scanlineAt(i);
            if (scanline.resolution == 0) {
                continue;
            }
            lcmHorizontalResolution = std::lcm(lcmHorizontalResolution, static_cast<uint32_t>(scanline.resolution));
        }

        RangeImage rangeImage(lcmHorizontalResolution, intrinsics.scanlinesCount(), 0);

        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            const uint32_t row = intrinsics.scanlinesCount() - minIndices(pointIdx) - 1;
            const double normalizedTheta =  correctedThetas(pointIdx) / (2 * std::numbers::pi);
            auto col = static_cast<int32_t>(std::round(normalizedTheta * lcmHorizontalResolution));

            col = col < 0 ? lcmHorizontalResolution - col : col;
            col = col >= lcmHorizontalResolution ? col - lcmHorizontalResolution : col;

            if (rangeImage(row, col) != 0) {
                LOG_WARN("Overwriting pixel at (", row, ", ", col, ") with range ", ranges(pointIdx),
                         " (previously: ", rangeImage(row, col), ")");
            }

            rangeImage(row, col) = ranges(pointIdx);
        }

        return rangeImage;
    }

    PointCloud::Double unProjectRangeImage(const Intrinsics &intrinsics, const RangeImage &image) {
        AliceArray<double> xs, ys, zs;

        for (int32_t row = 0; row < image.height(); ++row) {
            const int32_t scanlineIdx = static_cast<int32_t>(image.height()) - row - 1;
            const double vOffset = intrinsics.scanlineAt(scanlineIdx).verticalOffset;
            const double vAngle = intrinsics.scanlineAt(scanlineIdx).verticalAngle;
            const double hOffset = intrinsics.scanlineAt(scanlineIdx).horizontalOffset;
            const double thetaOffset = intrinsics.scanlineAt(scanlineIdx).azimuthalOffset;

            for (int32_t col = 0; col < image.width(); ++col) {
                const double range = image(row, col);

                if (range <= 0) {
                    continue;
                }

                const double originalPhi = vAngle + vOffset / range;
                const double rangeXy = range * std::cos(originalPhi);
                double originalTheta = col * Constant::TWO_PI / image.width() - std::numbers::pi;
                originalTheta += hOffset / rangeXy + thetaOffset;
                originalTheta = Utils::positiveFmod(originalTheta, Constant::TWO_PI);

                xs.emplace_back(rangeXy * std::cos(originalTheta));
                ys.emplace_back(rangeXy * std::sin(originalTheta));
                zs.emplace_back(range * std::sin(originalPhi));
            }
        }

        return PointCloud::Double(std::move(xs), std::move(ys), std::move(zs));
    }
}
