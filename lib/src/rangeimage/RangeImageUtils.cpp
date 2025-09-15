#include "RangeImageUtils.h"
#include <numbers>
#include <numeric>
#include <span>
#include <Eigen/Core>
#include "accurate_ri/public_structs.hpp"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"
#include "Constants.h"

namespace accurate_ri::RangeImageUtils {
    template<typename Scalar>
    RangeImage computeRangeImage(
        const Intrinsics &intrinsics, const Eigen::ArrayX<Scalar> &x, const Eigen::ArrayX<Scalar> &y,
        const Eigen::ArrayX<Scalar> &z
    );

    template<typename Scalar>
    int32_t findBestScanline(
        std::span<Scanline const> scanlines, const Eigen::ArrayX<Scalar> &ranges, const Eigen::ArrayX<Scalar> &phis, int32_t pointIdx
    );

    template<typename Scalar>
    RangeImage buildProjection(
        const Intrinsics &intrinsics, const Eigen::ArrayXi &scanlinesByPoints, const Eigen::ArrayX<Scalar> &ranges,
        const Eigen::ArrayXd &correctedThetas
    );

    inline int32_t calculateLcmHorizontalResolution(const Intrinsics &intrinsics);

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) {
        PROFILE_SCOPE("RangeImageUtils::projectToRangeImage");
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXf x = Eigen::Map<const Eigen::ArrayXf>(points.x.data(), size);
        const Eigen::ArrayXf y = Eigen::Map<const Eigen::ArrayXf>(points.y.data(), size);
        const Eigen::ArrayXf z = Eigen::Map<const Eigen::ArrayXf>(points.z.data(), size);

        return computeRangeImage(intrinsics, x, y, z);
    }

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) {
        PROFILE_SCOPE("RangeImageUtils::projectToRangeImage");
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd x = Eigen::Map<const Eigen::ArrayXd>(points.x.data(), size);
        const Eigen::ArrayXd y = Eigen::Map<const Eigen::ArrayXd>(points.y.data(), size);
        const Eigen::ArrayXd z = Eigen::Map<const Eigen::ArrayXd>(points.z.data(), size);

        return computeRangeImage(intrinsics, x, y, z);
    }

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image) {
        PointCloud::Double result;
        const double *const rangeImageData = image.data();
        const Scanline* const scanlines = intrinsics.scanlines.data();

        for (int32_t row = 0; row < image.height(); ++row) {
            const int32_t scanlineIdx = static_cast<int32_t>(image.height()) - row - 1;
            const Scanline &scanline = scanlines[scanlineIdx];

            for (int32_t col = 0; col < image.width(); ++col) {
                const uint32_t flatIdx = row * image.width() + col;
                const double range = rangeImageData[flatIdx];

                if (range <= 0) {
                    continue;
                }

                const double originalPhi = scanline.verticalAngle + scanline.verticalOffset / range;
                const double rangeXy = range * std::cos(originalPhi);
                double originalTheta = col * Constant::TWO_PI / image.width() - std::numbers::pi;
                originalTheta += scanline.horizontalOffset / rangeXy + scanline.azimuthalOffset;
                originalTheta = Utils::positiveFmod(originalTheta, Constant::TWO_PI);

                result.x.emplace_back(rangeXy * std::cos(originalTheta));
                result.y.emplace_back(rangeXy * std::sin(originalTheta));
                result.z.emplace_back(range * std::sin(originalPhi));
            }
        }

        return result;
    }

    template<typename Scalar>
    RangeImage computeRangeImage(
        const Intrinsics &intrinsics, const Eigen::ArrayX<Scalar> &x, const Eigen::ArrayX<Scalar> &y,
        const Eigen::ArrayX<Scalar> &z
    ) {
        const Eigen::ArrayX<Scalar> rangesXySquared = x.square() + y.square();
        const Eigen::ArrayX<Scalar> ranges = (rangesXySquared + z.square()).sqrt();
        const Eigen::ArrayX<Scalar> rangesXy = rangesXySquared.sqrt();
        const Eigen::ArrayX<Scalar> phis = (z / ranges).asin();
        const Eigen::ArrayX<Scalar> thetas = y.binaryExpr(x, [](const Scalar yi, const Scalar xi) {
            return static_cast<Scalar>(std::atan2(yi, xi) + std::numbers::pi);
        });
        Eigen::ArrayXd correctedThetas(phis.size());
        Eigen::ArrayXi scanlinesByPoints(phis.size());

        const Scanline* const scanlinesData = intrinsics.scanlines.data();
        const uint64_t scanlinesCount = intrinsics.scanlines.size();
        const std::span scanlines(scanlinesData, scanlinesCount);

        for (int32_t pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            const int32_t bestScanlineIdx = findBestScanline(scanlines, ranges, phis, pointIdx);

            scanlinesByPoints(pointIdx) = bestScanlineIdx;
            const double hOffset = scanlines[bestScanlineIdx].horizontalOffset;
            const double thetaOffset = scanlines[bestScanlineIdx].azimuthalOffset;
            correctedThetas(pointIdx) = thetas(pointIdx) - hOffset / rangesXy(pointIdx) - thetaOffset;
        }

        Utils::positiveFmodInplace(correctedThetas, Constant::TWO_PI);

        return buildProjection(intrinsics, scanlinesByPoints, ranges, correctedThetas);
    }

    template<typename Scalar>
    int32_t findBestScanline(
        const std::span<Scanline const> scanlines, const Eigen::ArrayX<Scalar> &ranges,
        const Eigen::ArrayX<Scalar> &phis, const int32_t pointIdx
    ) {
        double minPhiDiff = std::numeric_limits<double>::max();
        int32_t bestScanlineIdx = -1;

        for (int32_t laserIdx = 0; laserIdx < scanlines.size(); ++laserIdx) {
            const double vOffset = scanlines[laserIdx].verticalOffset;
            const double vAngle = scanlines[laserIdx].verticalAngle;

            const double phiCorrection = vOffset / ranges(pointIdx);
            const double phiDiff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

            if (phiDiff < minPhiDiff) {
                minPhiDiff = phiDiff;
                bestScanlineIdx = laserIdx;
            }
        }

        return bestScanlineIdx;
    }

    template<typename Scalar>
    RangeImage buildProjection(
        const Intrinsics &intrinsics, const Eigen::ArrayXi &scanlinesByPoints, const Eigen::ArrayX<Scalar> &ranges,
        const Eigen::ArrayXd &correctedThetas
    ) {
        const int32_t width = calculateLcmHorizontalResolution(intrinsics);
        const int32_t height = static_cast<int32_t>(intrinsics.scanlines.size());
        RangeImage rangeImage(width, height, 0);
        double *rangeImageData = rangeImage.data();

        for (int32_t pointIdx = 0; pointIdx < ranges.size(); ++pointIdx) {
            const int32_t row = height - scanlinesByPoints(pointIdx) - 1;
            const double normalizedTheta =  correctedThetas(pointIdx) / (2 * std::numbers::pi);
            auto col = static_cast<int32_t>(std::round(normalizedTheta * width));

            col = col < 0 ? width - col : col;
            col = col >= width ? col - width : col;

            const int32_t flatIdx = row * width + col;

            if (rangeImageData[flatIdx] != 0) {
                LOG_WARN("Overwriting pixel at (", row, ", ", col, ") with range ", ranges(pointIdx),
                         " (previously: ", rangeImage(row, col), "). Losslessness not achieved!");
            }

            rangeImageData[flatIdx] = ranges(pointIdx);
        }

        return rangeImage;
    }

    inline int32_t calculateLcmHorizontalResolution(const Intrinsics &intrinsics) {
        int32_t result = 1;
        for (const auto & scanline : intrinsics.scanlines) {
            if (scanline.resolution == 0) {
                continue;
            }
            result = static_cast<int32_t>(std::lcm(result, static_cast<uint32_t>(scanline.resolution)));
        }

        return result;
    }
}
