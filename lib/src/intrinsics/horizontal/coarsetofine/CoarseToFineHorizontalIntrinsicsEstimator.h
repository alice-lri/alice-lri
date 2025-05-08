#pragma once
#include <optional>
#include <unordered_set>

#include "accurate_ri/public_structs.hpp"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "point/PointArray.h"

namespace accurate_ri {
    struct ResolutionOffsetLoss {
        int32_t resolution;
        double offset;
        double loss;
    };

    class CoarseToFineHorizontalIntrinsicsEstimator {
    public:
        HorizontalIntrinsicsResult estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);

    private:
        int32_t optimizeResolutionCoarse(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx);

        std::pair<double, double> optimizeOffsetCoarse(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution
        );

        std::optional<ResolutionOffsetLoss> optimizeJoint(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t initialResInt
        );

        std::pair<double, double> optimizeOffsetPrecise(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution, double offsetGuess
        );

        double computeCoarseLoss(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution
        );

        double computePreciseLoss(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution
        );

        void updateHeuristicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );

        std::pair<int32_t, double> optimizeFromCandidatesPrecise(
            const Eigen::ArrayXd &thetas,
            const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResInts,
            const std::vector<double> &candidateOffsets
        );

        [[nodiscard]] double computeWeightedAverageSlope(
            const Eigen::ArrayXd &diffDiffToIdeal,
            const Eigen::ArrayXd &diffInvRangesXY,
            const Eigen::ArrayX<bool> &nonJumpMask
        ) const;

        int32_t madOptimalResolution(const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx);
    };
} // accurate_ri
