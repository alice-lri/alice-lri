#pragma once
#include "hough/HoughTransform.h"

namespace accurate_ri {
    class VerticalScanlinePool {
    private:
        HoughTransform hough;
        std::unordered_map<uint32_t, ScanlineInfo> scanlineInfoMap;
        Eigen::ArrayXi pointsScanlinesIds;

        int64_t unassignedPoints = 0;

    public:
        VerticalScanlinePool::VerticalScanlinePool(
            const double offsetMin, double offsetMax, double offsetStep, double angleMin, double angleMax,
            double angleStep
        );

        void performPrecomputations(const PointArray &points);

        std::optional<HoughScanlineEstimation> performHoughEstimation();

        inline void invalidateHash(uint64_t hash);

        inline bool anyUnassigned() const;

        [[nodiscard]] double getXMin() const { return hough.getXMin(); }

        [[nodiscard]] double getXMax() const { return hough.getXMax(); }

        [[nodiscard]] double getXStep() const { return hough.getXStep(); }

        [[nodiscard]] double getYMin() const { return hough.getYMin(); }

        [[nodiscard]] double getYMax() const { return hough.getYMax(); }

        [[nodiscard]] double getYStep() const { return hough.getYStep(); }

        [[nodiscard]] uint32_t getXCount() const { return hough.getXCount(); }

        [[nodiscard]] uint32_t getYCount() const { return hough.getYCount(); }
    };
} // accurate_ri
