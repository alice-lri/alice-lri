#pragma once
#include <unordered_set>

#include "accurate_ri/public_structs.hpp"
#include "point/PointArray.h"

namespace accurate_ri {
    class CoarseToFineHorizontalIntrinsicsEstimator {
    public:
        HorizontalIntrinsicsResult estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);

    private:
        int32_t optimizeResolutionCoarse(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges);

        std::pair<double, double> optimizeOffsetCoarse(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution
        );

        int32_t refineResolutionPrecise(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, int32_t initialResInt, double offset
        );

        std::pair<double, double> optimizeOffsetPrecise(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution
        );

        double computeCoarseLoss(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution
        );

        double computePreciseLoss(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution
        );

        void updateHeuristicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<uint32_t> &heuristicScanlines,
            const std::vector<Eigen::ArrayXd> &rangesXyByScanline, const std::vector<Eigen::ArrayXd> &thetasByScanline
        );

        std::pair<int32_t, double> optimizeFromCandidatesPrecise(
            const Eigen::ArrayXd &thetas,
            const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResInts,
            const std::vector<double> &candidateOffsets
        );
    };
} // accurate_ri
