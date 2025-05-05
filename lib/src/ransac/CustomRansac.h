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

        std::optional<CustomRansacResult> fit(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, std::optional<double> offsetGuess);

    private:
        bool refineFit(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &weights,
            const HorizontalScanlineArray &
            scanlineArray, int32_t scanlineIdx
        );

        bool fitToBoundsQp(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &eps, const double slope,
            const double intercept, double &deltaSlopeOut, double &deltaInterceptOut
        );
    };
} // accurate_ri
