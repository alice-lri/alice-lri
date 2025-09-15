#pragma once
#include <optional>
#include <unordered_set>

#include "intrinsics/horizontal/HorizontalIntrinsicsStructs.h"
#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "point/PointArray.h"

namespace alice_lri {
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
        static HorizontalIntrinsicsEstimation estimate(const PointArray &points, const VerticalIntrinsicsEstimation &vertical);

    private:
        static std::optional<HorizontalScanline> estimateScanline(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx
        );

        static std::optional<ResolutionOffsetLoss> findOptimalHorizontalParameters(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx
        );

        static ResolutionOffsetLoss optimizeJointCandidateResolution(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, int32_t resolution
        );

        static ResolutionOffsetLoss computeHeuristicValues(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, int32_t resolution, double offset
        );

        static void updateHeuristicScanlines(
            std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );

        static std::unordered_set<int32_t> getUniqueResolutions(
            const std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        static std::unordered_set<double> getUniqueOffsets(
            const std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines
        );

        static ResolutionOffsetLoss optimizeFromCandidatesHeuristic(
            const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges,
            const std::unordered_set<int32_t> &candidateResolutions, const std::unordered_set<double> &candidateOffsets
        );

        static void updateBasicScanlines(
            std::vector<HorizontalScanline> &scanlines, const std::unordered_set<int32_t> &heuristicScanlines,
            const HorizontalScanlineArray &scanlineArray
        );
    };
}
