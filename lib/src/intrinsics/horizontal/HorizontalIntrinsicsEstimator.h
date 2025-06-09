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

        ResolutionOffsetLoss(const int32_t resolution, const double offset, const double loss)
            : resolution(resolution), offset(offset), loss(loss) {}
    };

    class HorizontalIntrinsicsEstimator {
    public:
        HorizontalIntrinsicsResult estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);

    private:
        std::optional<ScanlineHorizontalInfo> estimateScanline(
            const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
        );

        std::optional<ResolutionOffsetLoss> optimizeJoint(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t initialResolution
        );

        std::vector<int32_t> generateCandidateResolutions(int32_t initialResolution, int32_t scanlineSize);

        ResolutionOffsetLoss optimizeJointCandidateResolution(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t resolution
        );

        double computePreciseLoss(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double thetaStep
        );

        void updateHeuristicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );

        std::unordered_set<int32_t> getUniqueResolutions(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        std::unordered_set<double> getUniqueOffsets(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        ResolutionOffsetLoss optimizeFromCandidatesPrecise(
            const Eigen::ArrayXd &thetas,
            const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResolutions,
            const std::unordered_set<double> &candidateOffsets
        );

        int32_t madOptimalResolution(const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx);
    };
} // accurate_ri
