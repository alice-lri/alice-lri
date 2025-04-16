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
        const int32_t resolution;

        CustomEstimator estimator;
        std::optional<Stats::LRResult> model = std::nullopt;

    public:
        CustomRansac(
            const uint32_t maxTrials, const int32_t resolution
        ) : maxTrials(maxTrials), resolution(resolution), estimator(2 * M_PI / resolution) {}

        std::optional<CustomRansacResult> fit(const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx);

    private:
        void refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);
    };
} // accurate_ri
