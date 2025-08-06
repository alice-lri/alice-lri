#include "PeriodicFit.h"

#include "utils/Logger.h"

// TODO reuse buffers and check performance
namespace accurate_ri {
    struct NewMultiLineResult {
        Eigen::ArrayXd residuals;
        Eigen::ArrayXi linesIdx;
    };

    constexpr int TRIG_TABLE_SIZE = 65536;

    const std::array<double, TRIG_TABLE_SIZE>& getSinTable() {
        static const std::array<double, TRIG_TABLE_SIZE> lut = [] {
            std::array<double, TRIG_TABLE_SIZE> table{};
            for (int i = 0; i < TRIG_TABLE_SIZE; ++i) {
                double angle = (2.0 * M_PI * i) / TRIG_TABLE_SIZE;
                table[i] = std::sin(angle);
            }
            return table;
        }();
        return lut;
    }

    const std::array<double, TRIG_TABLE_SIZE>& getCosTable() {
        static const std::array<double, TRIG_TABLE_SIZE> lut = [] {
            std::array<double, TRIG_TABLE_SIZE> table{};
            for (int i = 0; i < TRIG_TABLE_SIZE; ++i) {
                double angle = (2.0 * M_PI * i) / TRIG_TABLE_SIZE;
                table[i] = std::cos(angle);
            }
            return table;
        }();
        return lut;
    }

    double computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, const double thetaStep) {
        constexpr double twoPi = 2.0 * M_PI;

        const auto& sinLUT = getSinTable();
        const auto& cosLUT = getCosTable();

        Eigen::ArrayXd residualsMod = residuals - thetaStep * (residuals / thetaStep).floor();

        residualsMod *= (TRIG_TABLE_SIZE / thetaStep);
        residualsMod = residualsMod.min(TRIG_TABLE_SIZE - 1);

        double sinSum = 0.0, cosSum = 0.0;
        for (const double residual : residualsMod) {
            const int idx = static_cast<int>(residual);
            sinSum += sinLUT[idx];
            cosSum += cosLUT[idx];
        }

        double circularMean = std::atan2(sinSum / residualsMod.size(), cosSum / residualsMod.size());

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
