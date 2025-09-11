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
    inline RangeImage computeRangeImage(
        const Intrinsics &intrinsics, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges, const Eigen::ArrayXd &rangesXy
    );
    inline int32_t findBestScanline(
        const Intrinsics& intrinsics, const Eigen::ArrayXd &ranges, const Eigen::ArrayXd &phis, int32_t pointIdx
    );
    inline RangeImage buildProjection(
        const Intrinsics &intrinsics, const Eigen::ArrayXi &scanlinesByPoints, const Eigen::ArrayXd &ranges,
        const Eigen::ArrayXd &correctedThetas
    );
    inline int32_t calculateLcmHorizontalResolution(const Intrinsics &intrinsics);

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) {
        PROFILE_SCOPE("RangeImageUtils::projectToRangeImage");
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const auto x = Eigen::Map<const Eigen::ArrayXf>(points.x.data(), size);
        const auto y = Eigen::Map<const Eigen::ArrayXf>(points.y.data(), size);
        const auto z = Eigen::Map<const Eigen::ArrayXf>(points.z.data(), size);

        const Eigen::ArrayXd rangesXySquared = (x.square() + y.square()).cast<double>();
        const Eigen::ArrayXd ranges = (rangesXySquared + z.square().cast<double>()).sqrt();
        const Eigen::ArrayXd rangesXy = rangesXySquared.sqrt();
        const Eigen::ArrayXd phis = (z.cast<double>() / ranges).asin();
        const Eigen::ArrayXd thetas = y.binaryExpr(x, [](const double yi, const double xi) {
            return std::atan2(yi, xi) + std::numbers::pi;
        });

        return computeRangeImage(intrinsics, phis, thetas, ranges, rangesXy);
    }

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) {
        PROFILE_SCOPE("RangeImageUtils::projectToRangeImage");
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const auto x = Eigen::Map<const Eigen::ArrayXd>(points.x.data(), size);
        const auto y = Eigen::Map<const Eigen::ArrayXd>(points.y.data(), size);
        const auto z = Eigen::Map<const Eigen::ArrayXd>(points.z.data(), size);

        const Eigen::ArrayXd rangesXySquared = x.square() + y.square();
        const Eigen::ArrayXd ranges = (rangesXySquared + z.square()).sqrt();
        const Eigen::ArrayXd rangesXy = rangesXySquared.sqrt();
        const Eigen::ArrayXd phis = (z / ranges).asin();
        const Eigen::ArrayXd thetas = y.binaryExpr(x, [](const double yi, const double xi) {
            return std::atan2(yi, xi) + std::numbers::pi;
        });

        return computeRangeImage(intrinsics, phis, thetas, ranges, rangesXy);
    }

    // TODO maybe compare to pre-loop once to count?
    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image) {
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

    inline RangeImage computeRangeImage(
        const Intrinsics &intrinsics, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &thetas,
        const Eigen::ArrayXd &ranges, const Eigen::ArrayXd &rangesXy
    ) {
        Eigen::ArrayXi scanlinesByPoints(phis.size());
        Eigen::ArrayXd correctedThetas(phis.size());

        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            const int32_t bestScanlineIdx = findBestScanline(intrinsics, ranges, phis, pointIdx);

            scanlinesByPoints(pointIdx) = bestScanlineIdx;
            const double hOffset = intrinsics.scanlineAt(bestScanlineIdx).horizontalOffset;
            const double thetaOffset = intrinsics.scanlineAt(bestScanlineIdx).azimuthalOffset;
            correctedThetas(pointIdx) = thetas(pointIdx) - hOffset / rangesXy(pointIdx) - thetaOffset;
        }

        Utils::positiveFmodInplace(correctedThetas, Constant::TWO_PI);

        return buildProjection(intrinsics, scanlinesByPoints, ranges, correctedThetas);
    }

    inline int32_t findBestScanline(
        const Intrinsics& intrinsics, const Eigen::ArrayXd &ranges, const Eigen::ArrayXd &phis, const int32_t pointIdx
    ) {
        double minPhiDiff = std::numeric_limits<double>::max();
        int32_t bestScanlineIdx = -1;

        for (int32_t laserIdx = 0; laserIdx < intrinsics.scanlinesCount(); ++laserIdx) {
            const double vOffset = intrinsics.scanlineAt(laserIdx).verticalOffset;
            const double vAngle = intrinsics.scanlineAt(laserIdx).verticalAngle;

            const double phiCorrection = vOffset / ranges(pointIdx);
            const double phiDiff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

            if (phiDiff < minPhiDiff) {
                minPhiDiff = phiDiff;
                bestScanlineIdx = laserIdx;
            }
        }

        return bestScanlineIdx;
    }

    inline RangeImage buildProjection(
        const Intrinsics &intrinsics, const Eigen::ArrayXi &scanlinesByPoints, const Eigen::ArrayXd &ranges,
        const Eigen::ArrayXd &correctedThetas
    ) {
        const int32_t lcmHorizontalResolution = calculateLcmHorizontalResolution(intrinsics);
        RangeImage rangeImage(lcmHorizontalResolution, intrinsics.scanlinesCount(), 0);

        for (int32_t pointIdx = 0; pointIdx < ranges.size(); ++pointIdx) {
            const uint32_t row = intrinsics.scanlinesCount() - scanlinesByPoints(pointIdx) - 1;
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

    inline int32_t calculateLcmHorizontalResolution(const Intrinsics &intrinsics) {
        int32_t result = 1;
        for (int i = 0; i < intrinsics.scanlinesCount(); ++i) {
            const auto &scanline = intrinsics.scanlineAt(i);
            if (scanline.resolution == 0) {
                continue;
            }
            result = static_cast<int32_t>(std::lcm(result, static_cast<uint32_t>(scanline.resolution)));
        }

        return result;
    }
}
