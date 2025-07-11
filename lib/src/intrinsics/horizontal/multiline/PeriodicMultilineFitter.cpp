#include "PeriodicMultilineFitter.h"
#include <limits>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include <vector>
#include "plotty/matplotlibcpp.hpp"

namespace accurate_ri {
    struct MultiLineItem {
        double xWeightedSum = 0;
        double yWeightedSum = 0;
        double weightSum = 0;

        inline void push(const double x, const double y, const double weight) {
            xWeightedSum += x * weight;
            yWeightedSum += y * weight;
            weightSum += weight;
        }

        [[nodiscard]] inline double xMean() const {
            return xWeightedSum / weightSum;
        }

        [[nodiscard]] inline double yMean() const {
            return yWeightedSum / weightSum;
        }
    };

    PeriodicMultilineFitResult PeriodicMultilineFitter::fit(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const Stats::LRResult &lrGuess
    ) {
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &x = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd y = HorizontalMath::computeDiffToIdeal(thetas, resolution, false); // TODO we are now recomputing this multiple times

        model->slope = lrGuess.slope;
        model->intercept = lrGuess.intercept;
        estimator.setModel(*model);

        // // TODO refactor this set model thing
        // model->intercept = computeCircularMeanIntercept(estimator.computeResiduals(x, y), 2 * M_PI / resolution);
        // estimator.setModel(*model);

        LOG_DEBUG("Two-point slope: ", model->slope);
        const double loss = refineFit(x, y);
        LOG_DEBUG("Refined slope: ", model->slope, " and intercept: ", model->intercept);
        return PeriodicMultilineFitResult {.model = *model, .loss = loss};
    }

    double PeriodicMultilineFitter::refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        // TODO this is done to update the multilineresult, but maybe it can be avoided
        estimator.computeResiduals(x, y);
        const MultiLineResult &multi = estimator.getLastMultiLine();

        const double thetaStep = 2 * M_PI / resolution;
        const Eigen::ArrayXd shiftedY = y - multi.linesIdx.cast<double>() * thetaStep;

        // TODO now here we could use the wls fit method and the uncertainty maybe
        // TODO maybe we are over-complicating, and we can infer this straight from the modulo residuals
        const Stats::LRResult fitResult = Stats::simpleLinearRegression(x, shiftedY, true);
        model = fitResult;
        estimator.setModel(*model);

        return *fitResult.mse;
    }

    double PeriodicMultilineFitter::computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, const double k) {
        constexpr double twoPi = 2.0 * M_PI;

        // Step 1: Map residuals mod k into [0, k)
        Eigen::ArrayXd mod = residuals.unaryExpr([k](double r) {
            double m = std::fmod(r, k);
            return m < 0 ? m + k : m;
        });

        // Step 2: Convert to angles in [0, 2π)
        Eigen::ArrayXd theta = (twoPi / k) * mod;

        // Step 3: Compute mean of cos and sin
        double x_sum = theta.cos().mean();
        double y_sum = theta.sin().mean();

        // Step 4: Compute circular mean angle
        double theta_bar = std::atan2(y_sum, x_sum);
        if (theta_bar < 0) theta_bar += twoPi;

        // Step 5: Convert back to b in [0, k)
        return (k * theta_bar) / twoPi;
    }
} // accurate_ri
