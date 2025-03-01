#include "HorizontalIntrinsicsEstimator.h"

#include "intrinsics/horizontal/resolution/MadResolutionLoss.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "utils/Logger.h"

namespace accurate_ri {
    template<typename T>
    int32_t computeOptimalResolution(const Eigen::ArrayBase<T> &invRangesXy, const Eigen::ArrayBase<T> &thetas) {
        // TODO do not hardcode these
        constexpr int32_t minResolution = 1000;
        constexpr int32_t maxResolution = 10000;

        double minLoss = std::numeric_limits<double>::infinity();
        int32_t optimalResolution = -1;

        for (int32_t resolution = minResolution; resolution < maxResolution; ++resolution) {
            const double loss = MadResolutionLoss::computeResolutionLoss(invRangesXy, thetas, resolution);

            if (loss < minLoss) {
                minLoss = loss;
                optimalResolution = resolution;
            }
        }

        return optimalResolution;
    }

    void HorizontalIntrinsicsEstimator::estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical) {
        std::vector<std::vector<int32_t>> pointsByScanline(vertical.scanlinesCount);

        for (int32_t pointIdx = 0; pointIdx < vertical.pointsCount; ++pointIdx) {
            const int32_t scanlineIdx = vertical.fullScanlines.pointsScanlinesIds(pointIdx);

            if (scanlineIdx >= 0) {
                pointsByScanline[scanlineIdx].emplace_back(pointIdx);
            }
        }

        for (auto &scanline : pointsByScanline) {
            std::ranges::sort(scanline, [&](const int32_t a, const int32_t b) {
                return points.getRangeXy(a) < points.getRangeXy(b);
            });
        }

        std::vector<Eigen::ArrayXd> invRangesXyByScanline;
        std::vector<Eigen::ArrayXd> thetasByScanline;

        invRangesXyByScanline.reserve(vertical.scanlinesCount);
        thetasByScanline.reserve(vertical.scanlinesCount);

        for (int scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            invRangesXyByScanline.emplace_back(points.getInvRangesXy()(pointsByScanline[scanlineIdx]));

            Eigen::ArrayXd scanlineThetas = points.getThetas()(pointsByScanline[scanlineIdx]);
            scanlineThetas -= scanlineThetas(0);
            thetasByScanline.emplace_back(scanlineThetas);
        }

        for (int scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            const auto &invRangesXy = invRangesXyByScanline[scanlineIdx];
            const auto &thetas = thetasByScanline[scanlineIdx];

            int32_t resolution = computeOptimalResolution(invRangesXy, thetas);
            LOG_INFO("Scanline ", scanlineIdx, " optimal resolution: ", resolution);
        }
    }
} // accurate_ri
