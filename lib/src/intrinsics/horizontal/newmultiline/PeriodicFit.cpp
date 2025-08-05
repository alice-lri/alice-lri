#include "PeriodicFit.h"

#include "utils/Logger.h"

// TODO reuse buffers and check performance
namespace accurate_ri {
    struct NewMultiLineResult {
        Eigen::ArrayXd residuals;
        Eigen::ArrayXi linesIdx;
    };

    NewMultiLineResult buffer;

    double computeCircularMeanIntercept(Eigen::ArrayXd& residuals, const double thetaStep) {
        constexpr double twoPi = 2.0 * M_PI;

        residuals = residuals.unaryExpr([thetaStep](const double residual) {
            const double residualMod = std::fmod(residual, thetaStep);
            return residualMod < 0 ? residualMod + thetaStep : residualMod;
        });

        residuals = (twoPi / thetaStep) * residuals;
        double circularMean = std::atan2(residuals.sin().mean(), residuals.cos().mean());

        if (circularMean < 0) {
            circularMean += twoPi;
        }

        return (thetaStep * circularMean) / twoPi;
    }

    Stats::LRResult refineFit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXi& linesIdx, const double period
    ) {
        const Eigen::ArrayXd shiftedY = y - linesIdx.cast<double>() * period;
        const Stats::LRResult fitResult = Stats::linearRegression(x, shiftedY, true);

        return fitResult;
    }

    void computePeriodicResiduals(
        const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, const double period, const double slope,
        const double intercept
    ) {
        buffer.residuals = y - (slope * x + intercept);
        buffer.linesIdx = (buffer.residuals / period).round().cast<int>();
        buffer.residuals = buffer.residuals - buffer.linesIdx.cast<double>() * period;
    }

    Stats::LRResult PeriodicFit::fit(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const double period, const double slopeGuess
    ) {
        computePeriodicResiduals(x, y, period, slopeGuess, 0);

        const double intercept = computeCircularMeanIntercept(buffer.residuals, period);
        computePeriodicResiduals(x, y, period, slopeGuess, intercept);

        return refineFit(x, y, buffer.linesIdx, period);
    }
}
