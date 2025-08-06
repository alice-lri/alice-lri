#include "PeriodicFit.h"

#include "utils/Logger.h"

// TODO reuse buffers and check performance
namespace accurate_ri {
    struct NewMultiLineResult {
        Eigen::ArrayXd residuals;
        Eigen::ArrayXi linesIdx;
    };

    double computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, const double thetaStep) {
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

    Stats::LRResult refineFit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const NewMultiLineResult &multiLineResult, const double period
    ) {
        const Eigen::ArrayXd shiftedY = y - multiLineResult.linesIdx.cast<double>() * period;
        const Stats::LRResult fitResult = Stats::linearRegression(x, shiftedY, true);

        return fitResult;
    }

    void computePeriodicResiduals(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const double period, const double slope,
        const double intercept, NewMultiLineResult& outResult
    ) {
        outResult.residuals = y - (slope * x + intercept);
        outResult.linesIdx = (outResult.residuals / period).round().cast<int>();
        outResult.residuals = outResult.residuals - outResult.linesIdx.cast<double>() * period;
    }

    Stats::LRResult PeriodicFit::fit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const double period, const double slopeGuess
    ) {
        NewMultiLineResult multiLineResult;
        computePeriodicResiduals(x, y, period, slopeGuess, 0, multiLineResult);

        const double intercept = computeCircularMeanIntercept(multiLineResult.residuals, period);
        computePeriodicResiduals(x, y, period, slopeGuess, intercept, multiLineResult);

        return refineFit(x, y, multiLineResult, period);
    }
}
