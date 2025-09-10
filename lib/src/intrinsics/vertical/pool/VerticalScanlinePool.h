#pragma once
#include "hough/HoughTransform.h"
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"

namespace accurate_ri {
    class VerticalScanlinePool {
    private:
        std::unordered_map<uint32_t, VerticalScanline> scanlineInfoMap;
        Eigen::ArrayXi pointsScanlinesIds;
        int64_t unassignedPoints = 0;

        HoughTransform hough;

    public:
        VerticalScanlinePool(
            double offsetMin, double offsetMax, double offsetStep, double angleMin, double angleMax, double angleStep
        );

        void performPrecomputations(const PointArray &points);

        std::optional<HoughScanlineEstimation> performHoughEstimation();

        void assignScanline(const VerticalScanline &scanline, const Eigen::ArrayXi &pointsIndices);

        std::optional<VerticalScanline> removeScanline(const PointArray &points, uint32_t scanlineId);

        VerticalScanlinesAssignations extractFullSortedScanlineAssignations();

        [[nodiscard]] VerticalMargin getHoughMargin() const;

        [[nodiscard]] std::vector<VerticalScanline> getUnsortedScanlinesCopy() const;

        Eigen::ArrayXi getScanlinesIds(const Eigen::ArrayXi &pointsIndices) const {
            return pointsScanlinesIds(pointsIndices);
        }

        void restoreByHash(const uint64_t hash, const int64_t votes) {
            hough.restoreVotes(hash, votes);
        }

        void invalidateByHash(const uint64_t hash) {
            hough.eraseByHash(hash);
        }

        void removeVotes(const PointArray &points, const Eigen::ArrayXi &indices) {
            hough.removeVotes(points, indices);
        }

        const VerticalScanline &getScanlineById(const uint32_t id) const {
            return scanlineInfoMap.at(id);
        }

        bool anyUnassigned() const {
            return unassignedPoints > 0;
        }

        int64_t getUnassignedPoints() const {
            return unassignedPoints;
        }

        const Eigen::ArrayXi &getPointsScanlinesIds() const {
            return pointsScanlinesIds;
        }

        template<typename Func>
        void forEachScanline(Func &&func) const {
            for (const auto &scanline: scanlineInfoMap) {
                func(scanline.second);
            }
        }

        [[nodiscard]] double getXMin() const {
            return hough.getXMin();
        }

        [[nodiscard]] double getXMax() const {
            return hough.getXMax();
        }

        [[nodiscard]] double getXStep() const {
            return hough.getXStep();
        }

        [[nodiscard]] double getYMin() const {
            return hough.getYMin();
        }

        [[nodiscard]] double getYMax() const {
            return hough.getYMax();
        }

        [[nodiscard]] double getYStep() const {
            return hough.getYStep();
        }

        [[nodiscard]] uint32_t getXCount() const {
            return hough.getXCount();
        }

        [[nodiscard]] uint32_t getYCount() const {
            return hough.getYCount();
        }

    private:
        Eigen::ArrayXi scanlineIdToPointsIndices(uint32_t scanlineId) const;

        std::vector<VerticalScanline> computeSortedScanlines() const;

        void updateScanlineIds(std::vector<VerticalScanline> sortedScanlines);
    };
} // accurate_ri
