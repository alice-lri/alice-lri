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
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx
        );

        std::optional<ResolutionOffsetLoss> findOptimalHorizontalParameters(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx
        );

        [[nodiscard]] static ResolutionOffsetLoss optimizeJointCandidateResolution(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t resolution
        );

        static ResolutionOffsetLoss computeHeuristicValues(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, int32_t resolution, double offset
        );

        void updateHeuristicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );

        static std::unordered_set<int32_t> getUniqueResolutions(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        static std::unordered_set<double> getUniqueOffsets(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        ResolutionOffsetLoss optimizeFromCandidatesHeuristic(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResolutions, const std::unordered_set<double> &candidateOffsets
        );

        static void updateBasicScanlines(
            std::vector<ScanlineHorizontalInfo> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );
    };
} // accurate_ri
