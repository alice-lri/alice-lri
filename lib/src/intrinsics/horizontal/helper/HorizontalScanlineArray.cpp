#include "HorizontalScanlineArray.h"

namespace accurate_ri {
    HorizontalScanlineArray::HorizontalScanlineArray(
        const PointArray &points, const std::vector<int> &pointsScanlinesIds, const int32_t scanlinesCount
    ): coordsEps(points.getCoordsEps()) {
        std::vector<std::vector<int32_t>> pointsByScanline(scanlinesCount);

        for (int i = 0; i < pointsScanlinesIds.size(); ++i) {
            const int32_t scanlineIdx = pointsScanlinesIds[i];

            if (scanlineIdx >= 0) {
                pointsByScanline[scanlineIdx].emplace_back(i);
            }
        }

        // TODO check if sorting is still necessary
        for (auto &scanline: pointsByScanline) {
            std::ranges::sort(
                scanline, [&](const int32_t a, const int32_t b) {
                    return points.getTheta(a) < points.getTheta(b);
                }
            );
        }

        scanlineSizes.reserve(scanlinesCount);
        xsByScanline.reserve(scanlinesCount);
        ysByScanline.reserve(scanlinesCount);
        rangesXyByScanline.reserve(scanlinesCount);
        invRangesXyByScanline.reserve(scanlinesCount);
        thetasByScanline.reserve(scanlinesCount);

        thetasUpperBoundsByScanline.reserve(scanlinesCount);
        rangesXyMinusBoundsByScanline.reserve(scanlinesCount);


        for (int scanlineIdx = 0; scanlineIdx < scanlinesCount; ++scanlineIdx) {
            const std::vector<int> scanlineIndices = pointsByScanline[scanlineIdx];

            scanlineSizes.emplace_back(scanlineIndices.size());
            xsByScanline.emplace_back(points.getXs()(scanlineIndices));
            ysByScanline.emplace_back(points.getYs()(scanlineIndices));
            rangesXyByScanline.emplace_back(points.getRangesXy()(scanlineIndices));
            invRangesXyByScanline.emplace_back(points.getInvRangesXy()(scanlineIndices));
            thetasByScanline.emplace_back(points.getThetas()(scanlineIndices));

            thetasUpperBoundsByScanline.emplace_back(points.getThetaUpperBound()(scanlineIndices));
            rangesXyMinusBoundsByScanline.emplace_back(points.getRangeXyMinusBound()(scanlineIndices));

            // TODO check if this is still necessary
            thetasByScanline[scanlineIdx] -= thetasByScanline[scanlineIdx].minCoeff();
        }
    }

    Eigen::ArrayXd HorizontalScanlineArray::getCorrectionBounds(
        const int32_t scanlineIdx, const double hOffset
    ) const {
        const Eigen::ArrayXd& rangesXyMinusBounds = rangesXyMinusBoundsByScanline[scanlineIdx];
        const Eigen::ArrayXd& rangesXy = rangesXyByScanline[scanlineIdx];

        return (std::abs(hOffset) / rangesXyMinusBounds).asin() - (std::abs(hOffset) / rangesXy).asin();
    }
}
