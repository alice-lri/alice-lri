#include "PeriodicMultilineFitter.h"
#include <limits>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include <vector>
#include "plotty/matplotlibcpp.hpp"
#include "utils/Utils.h"

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
        model->intercept = 0;
        estimator.setModel(*model);

        // // TODO refactor this set model thing
        model->intercept = computeCircularMeanIntercept(estimator.computeResiduals(x, y), 2 * M_PI / resolution);
        estimator.setModel(*model);

        LOG_DEBUG("Circular intercept: ", model->intercept);
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
        const Stats::LRResult fitResult = Stats::linearRegression(x, shiftedY, true);
        model = fitResult;
        estimator.setModel(*model);

        return *fitResult.mse;
    }

    double PeriodicMultilineFitter::computeCircularMeanIntercept(
        const Eigen::ArrayXd& residuals, const double thetaStep
    ) {
        constexpr double twoPi = 2.0 * M_PI;

        Eigen::ArrayXd residualsMod = residuals.unaryExpr([thetaStep](const double residual) {
            const double residualMod = std::fmod(residual, thetaStep);
            return residualMod < 0 ? residualMod + thetaStep : residualMod;
        });

        residualsMod = (twoPi / thetaStep) * residualsMod;
        double circularMean = std::atan2(residualsMod.sin().mean(), residualsMod.cos().mean());

        if (circularMean < 0) {
            circularMean += twoPi;
        }

        return (thetaStep * circularMean) / twoPi;
    }
} // accurate_ri
