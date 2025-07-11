#pragma once
#include <optional>
#include <Eigen/Core>

#include "intrinsics/horizontal/helper/HorizontalScanlineArray.h"
#include "intrinsics/horizontal/multiline/CustomEstimator.h"
#include "math/Stats.h"

// TODO thetaStep is basically resolution, find better namings
namespace accurate_ri {
    struct PeriodicMultilineFitResult {
        Stats::LRResult model;
        double loss;
    };

    class PeriodicMultilineFitter {
    private:
        const int32_t resolution; // TODO probably name differently (algorithm is business independent)

        CustomEstimator estimator;
        std::optional<Stats::LRResult> model = std::nullopt;

    public:
        explicit PeriodicMultilineFitter(const int32_t resolution) : resolution(resolution), estimator(2 * M_PI / resolution) {}

        PeriodicMultilineFitResult fit(
            const HorizontalScanlineArray &scanlineArray, int32_t scanlineIdx, const Stats::LRResult &lrGuess
        );

    private:
        double refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y);

        double computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, double k);
    };
} // accurate_ri
