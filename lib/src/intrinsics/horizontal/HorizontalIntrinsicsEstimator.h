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
        double thetaOffset;
        double loss;

        ResolutionOffsetLoss(const int32_t resolution, const double offset, const double thetaOffset, const double loss)
            : resolution(resolution), offset(offset), thetaOffset(thetaOffset), loss(loss) {}
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

        std::vector<int32_t> generateCandidateResolutionsMad(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx);

        ResolutionOffsetLoss optimizeJointCandidateResolution(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t resolution
        );

        ResolutionOffsetLoss computeHeuristicValues(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, int32_t resolution, double offset
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

        ResolutionOffsetLoss optimizeFromCandidatesHeuristic(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResolutions, const std::unordered_set<double> &candidateOffsets
        );

        int32_t madOptimalResolution(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx);

        void updateBasicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );
    };
} // accurate_ri
