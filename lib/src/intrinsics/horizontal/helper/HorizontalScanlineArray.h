#pragma once
#include "point/PointArray.h"

namespace alice_lri {
    enum class SortingCriteria {
        RANGES_XY, THETAS, NONE
    };

    class HorizontalScanlineArray {
    private:
        std::vector<int32_t> scanlineSizes;
        std::vector<Eigen::ArrayXd> xsByScanline;
        std::vector<Eigen::ArrayXd> ysByScanline;
        std::vector<Eigen::ArrayXd> rangesXyByScanline;
        std::vector<Eigen::ArrayXd> invRangesXyByScanline;
        std::vector<Eigen::ArrayXd> thetasByScanline;

        std::vector<Eigen::ArrayXd> invRangesXyDiffByScanline;

    public:
        HorizontalScanlineArray(
            const PointArray &points, const std::vector<int> &pointsScanlinesIds, int32_t scanlinesCount,
            SortingCriteria sortingCriteria
        );

        [[nodiscard]] const int32_t &getSize(const int32_t scanlineIdx) const {
            return scanlineSizes[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getXs(const int32_t scanlineIdx) const {
            return xsByScanline[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getYs(const int32_t scanlineIdx) const {
            return ysByScanline[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getRangesXy(const int32_t scanlineIdx) const {
            return rangesXyByScanline[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getInvRangesXy(const int32_t scanlineIdx) const {
            return invRangesXyByScanline[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getThetas(const int32_t scanlineIdx) const {
            return thetasByScanline[scanlineIdx];
        }

        [[nodiscard]] const Eigen::ArrayXd &getInvRangesXyDiff(const int32_t scanlineIdx) const {
            return invRangesXyDiffByScanline[scanlineIdx];
        }

    private:
        static void sortScanlinePointsByCriteria(
            const PointArray &points, SortingCriteria sortingCriteria,
            std::vector<std::vector<int32_t>> &pointsByScanline
        );

        static std::vector<std::vector<int32_t>> groupPointsByScanline(
            const std::vector<int> &pointsScanlinesIds, int32_t scanlinesCount
        );

        void initScanlines(int32_t scanlinesCount);

        void populateScanlines(
            const PointArray &points, int32_t scanlinesCount, const std::vector<std::vector<int32_t>> &pointsByScanline
        );
    };
}
