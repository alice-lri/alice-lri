#pragma once
#include <cstdint>
#include <optional>
#include <Eigen/Core>

#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "math/Stats.h"
#include "ransac/CustomEstimator.h"

// TODO thetaStep is basically resolution, find better namings
namespace accurate_ri {
    struct CustomRansacResult {
        Stats::LRResult model;
        double loss;
    };

    class CustomRansac {
    private:
        const uint32_t maxTrials;
        const uint32_t maxFitToBoundsIterations;
        const int32_t resolution;

        CustomEstimator estimator;
        std::optional<Stats::LRResult> model = std::nullopt;

    public:
        CustomRansac(
            const uint32_t maxTrials, const uint32_t maxFitToBoundsIterations, const int32_t resolution
        ) : maxTrials(maxTrials),
            maxFitToBoundsIterations(maxFitToBoundsIterations),
            resolution(resolution),
            estimator(2 * M_PI / resolution) {}

        std::optional<CustomRansacResult> fit(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx);

    private:
        void refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &weights);

        std::optional<double> fitToBounds(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const HorizontalScanlineArray &scanlineArray,
            int32_t scanlineIdx
        );

        void fitToBoundsModifyIntercept(
            const Eigen::ArrayXd &residuals, const Eigen::ArrayXd &residualBounds, const Eigen::ArrayXi &outlierIndices
        );

        void fitToBoundsModifySlope(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &residuals, const Eigen::ArrayXd &residualBounds,
            const Eigen::ArrayXi &outlierIndices, const double pivotPoint
        );
    };
} // accurate_ri
