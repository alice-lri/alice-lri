#pragma once
#include "hough/HoughTransform.h"
#include "utils/Logger.h"

namespace accurate_ri {
    class VerticalScanlinePool {
    private:
        std::unordered_map<uint32_t, ScanlineInfo> scanlineInfoMap;
        Eigen::ArrayXi pointsScanlinesIds;
        int64_t unassignedPoints = 0;

        HoughTransform hough;

    public:
        VerticalScanlinePool(
            double offsetMin, double offsetMax, double offsetStep, double angleMin, double angleMax, double angleStep
        );

        void performPrecomputations(const PointArray &points);

        std::optional<HoughScanlineEstimation> performHoughEstimation();

        void assignScanline(const ScanlineInfo &scanline, const Eigen::ArrayXi &pointsIndices);

        std::optional<ScanlineInfo> removeScanline(const PointArray &points, uint32_t scanlineId);

        FullScanlines extractFullSortedScanlines();

        inline Eigen::ArrayXi getScanlinesIds(const Eigen::ArrayXi &pointsIndices) const {
            return pointsScanlinesIds(pointsIndices);
        }

        inline void restoreByHash(const uint64_t hash, const double votes) {
            hough.restoreVotes(hash, votes);
        }

        inline void invalidateByHash(const uint64_t hash) {
            hough.eraseByHash(hash);
        }

        inline void invalidateByPoints(const PointArray &points, const Eigen::ArrayXi &indices) {
            hough.eraseByPoints(points, indices);
        }

        inline void removeVotes(const PointArray& points, const Eigen::ArrayXi &indices) {
            hough.removeVotes(points, indices);
        }

        inline void addVotes(const PointArray& points, const Eigen::ArrayXi &indices) {
            hough.addVotes(points, indices);
        }

        inline const ScanlineInfo& getScanlineById(const uint32_t id) const {
            return scanlineInfoMap.at(id);
        }

        inline bool anyUnassigned() const { return unassignedPoints > 0; }

        inline int64_t getUnassignedPoints() const { return unassignedPoints; }

        inline const Eigen::ArrayXi& getPointsScanlinesIds() const { return pointsScanlinesIds; }

        template<typename Func>
        void forEachScanline(Func&& func) const {
            for (const auto &scanline: scanlineInfoMap) {
                func(scanline.second);
            }
        }

        [[nodiscard]] OffsetAngleMargin getHoughMargin() const;

        [[nodiscard]] std::vector<ScanlineInfo> getUnsortedScanlinesCopy() const;

        [[nodiscard]] double getXMin() const { return hough.getXMin(); }

        [[nodiscard]] double getXMax() const { return hough.getXMax(); }

        [[nodiscard]] double getXStep() const { return hough.getXStep(); }

        [[nodiscard]] double getYMin() const { return hough.getYMin(); }

        [[nodiscard]] double getYMax() const { return hough.getYMax(); }

        [[nodiscard]] double getYStep() const { return hough.getYStep(); }

        [[nodiscard]] uint32_t getXCount() const { return hough.getXCount(); }

        [[nodiscard]] uint32_t getYCount() const { return hough.getYCount(); }

        // TODO remove
        void debugHough() {
            hough.debugHough();
        }
    };
} // accurate_ri
