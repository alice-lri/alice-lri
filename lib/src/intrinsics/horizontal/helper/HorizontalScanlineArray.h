#pragma once
#include <Eigen/Dense>
#include "point/PointArray.h"

namespace accurate_ri {
    class HorizontalScanlineArray {
    private:
        std::vector<int32_t> scanlineSizes;
        std::vector<Eigen::ArrayXd> xsByScanline;
        std::vector<Eigen::ArrayXd> ysByScanline;
        std::vector<Eigen::ArrayXd> rangesXyByScanline;
        std::vector<Eigen::ArrayXd> invRangesXyByScanline;
        std::vector<Eigen::ArrayXd> thetasByScanline;

    public:
        HorizontalScanlineArray(
            const PointArray &points, const std::vector<int> &pointsScanlinesIds, const int32_t scanlinesCount
        );

        [[nodiscard]] inline const int32_t &getSize(const int32_t scanlineIdx) const {
            return scanlineSizes[scanlineIdx];
        }

        [[nodiscard]] inline const Eigen::ArrayXd &getXs(const int32_t scanlineIdx) const {
            return xsByScanline[scanlineIdx];
        }

        [[nodiscard]] inline const Eigen::ArrayXd &getYs(const int32_t scanlineIdx) const {
            return ysByScanline[scanlineIdx];
        }

        [[nodiscard]] inline const Eigen::ArrayXd &getRangesXy(const int32_t scanlineIdx) const {
            return rangesXyByScanline[scanlineIdx];
        }

        [[nodiscard]] inline const Eigen::ArrayXd &getInvRangesXy(const int32_t scanlineIdx) const {
            return invRangesXyByScanline[scanlineIdx];
        }

        [[nodiscard]] inline const Eigen::ArrayXd &getThetas(const int32_t scanlineIdx) const {
            return thetasByScanline[scanlineIdx];
        }
    };
}
