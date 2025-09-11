#include "PeriodicFitter.h"

#include "Constants.h"
#include "math/Trigonometry.h"

namespace accurate_ri {

    LRResult PeriodicFitter::fit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const double period, const double slopeGuess
    ) {
        NewMultiLineResult multiLineResult;
        computePeriodicResiduals(x, y, period, slopeGuess, 0, multiLineResult);

        const double intercept = computeCircularMeanIntercept(multiLineResult.residuals, period);
        computePeriodicResiduals(x, y, period, slopeGuess, intercept, multiLineResult);

        return refineFit(x, y, multiLineResult, period);
    }

    void PeriodicFitter::computePeriodicResiduals(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const double period, const double slope,
        const double intercept, NewMultiLineResult& outResult
    ) {
        outResult.residuals = y - (slope * x + intercept);
        outResult.linesIdx = (outResult.residuals / period).round().cast<int>();
        outResult.residuals = outResult.residuals - outResult.linesIdx.cast<double>() * period;
    }

    double PeriodicFitter::computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, const double period) {
        Eigen::ArrayXd residualsMod = residuals - period * (residuals / period).floor();
        residualsMod *= Trigonometry::TRIG_TABLE_SIZE / period;
        residualsMod = residualsMod.min(Trigonometry::TRIG_TABLE_SIZE - 1);

        double sinSum = 0.0, cosSum = 0.0;
        for (const double residual : residualsMod) {
            const int idx = static_cast<int>(residual);
            sinSum += Trigonometry::sinIndex(idx);
            cosSum += Trigonometry::cosIndex(idx);
        }

        sinSum /= static_cast<double>(residualsMod.size());
        cosSum /= static_cast<double>(residualsMod.size());
        double circularMean = std::atan2(sinSum, cosSum);

        if (circularMean < 0) {
            circularMean += Constant::TWO_PI;
        }

        return (period * circularMean) / Constant::TWO_PI;
    }

    LRResult PeriodicFitter::refineFit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, NewMultiLineResult &multiLineResult, const double period
    ) {
        const int32_t halfSize = static_cast<int32_t>(x.size()) / 2;
        Eigen::ArrayXd shiftedY = y - multiLineResult.linesIdx.cast<double>() * period;

        const LRResult fitResultFirst = LinearRegressor::fit(x.head(halfSize), shiftedY.head(halfSize), true);
        const LRResult fitResultLast = LinearRegressor::fit(x.tail(halfSize), shiftedY.tail(halfSize), true);
        const LRResult fitResultAll = LinearRegressor::fit(x, shiftedY, true);
        const LRResult& optFit = (fitResultFirst.mse < fitResultLast.mse) ? fitResultFirst : fitResultLast;

        computePeriodicResiduals(x, y, period, optFit.slope, optFit.intercept, multiLineResult);
        shiftedY = y - multiLineResult.linesIdx.cast<double>() * period;
        const LRResult fitResultFinal = LinearRegressor::fit(x, shiftedY, true);

        return fitResultAll.mse < fitResultFinal.mse ? fitResultAll : fitResultFinal;
    }
}
