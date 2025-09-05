#include "HorizontalScanlineArray.h"
#include "utils/Utils.h"

namespace accurate_ri {
    HorizontalScanlineArray::HorizontalScanlineArray(
        const PointArray &points, const std::vector<int> &pointsScanlinesIds, const int32_t scanlinesCount,
        const SortingCriteria sortingCriteria
    ) {
        auto pointsByScanline = groupPointsByScanline(pointsScanlinesIds, scanlinesCount);

        sortScanlinePointsByCriteria(points, sortingCriteria, pointsByScanline);
        initScanlines(scanlinesCount);
        populateScanlines(points, scanlinesCount, pointsByScanline);
    }

    std::vector<std::vector<int32_t>> HorizontalScanlineArray::groupPointsByScanline(
        const std::vector<int> &pointsScanlinesIds, const int32_t scanlinesCount
    ) {
        std::vector<std::vector<int32_t>> pointsByScanline(scanlinesCount);

        for (int i = 0; i < pointsScanlinesIds.size(); ++i) {
            const int32_t scanlineIdx = pointsScanlinesIds[i];

            if (scanlineIdx >= 0) {
                pointsByScanline[scanlineIdx].emplace_back(i);
            }
        }

        return pointsByScanline;
    }

    void HorizontalScanlineArray::sortScanlinePointsByCriteria(
        const PointArray &points, const SortingCriteria sortingCriteria,
        std::vector<std::vector<int32_t>> &pointsByScanline
    ) {
        if (sortingCriteria == SortingCriteria::NONE) {
            return;
        }

        auto comparator = [&](const int32_t a, const int32_t b) {
            switch (sortingCriteria) {
                case SortingCriteria::RANGES_XY:
                    return points.getRangeXy(a) < points.getRangeXy(b);
                case SortingCriteria::THETAS:
                    return points.getTheta(a) < points.getTheta(b);
                default:
                    throw std::invalid_argument("Invalid sorting criteria");
            }
        };

        for (auto &scanline: pointsByScanline) {
            std::ranges::sort(scanline, comparator);
        }
    }

    void HorizontalScanlineArray::initScanlines(const int32_t scanlinesCount) {
        scanlineSizes.reserve(scanlinesCount);
        xsByScanline.reserve(scanlinesCount);
        ysByScanline.reserve(scanlinesCount);
        rangesXyByScanline.reserve(scanlinesCount);
        invRangesXyByScanline.reserve(scanlinesCount);
        thetasByScanline.reserve(scanlinesCount);

        invRangesXyDiffByScanline.reserve(scanlinesCount);
    }

    void HorizontalScanlineArray::populateScanlines(
        const PointArray &points, const int32_t scanlinesCount,
        const std::vector<std::vector<int32_t>> &pointsByScanline
    ) {
        for (int scanlineIdx = 0; scanlineIdx < scanlinesCount; ++scanlineIdx) {
            const std::vector<int> &scanlineIndices = pointsByScanline[scanlineIdx];

            scanlineSizes.emplace_back(scanlineIndices.size());
            xsByScanline.emplace_back(points.getXs()(scanlineIndices));
            ysByScanline.emplace_back(points.getYs()(scanlineIndices));
            rangesXyByScanline.emplace_back(points.getRangesXy()(scanlineIndices));
            invRangesXyByScanline.emplace_back(points.getInvRangesXy()(scanlineIndices));
            thetasByScanline.emplace_back(points.getThetas()(scanlineIndices));

            invRangesXyDiffByScanline.emplace_back(Utils::diff(getInvRangesXy(scanlineIdx)));
        }
    }
}
