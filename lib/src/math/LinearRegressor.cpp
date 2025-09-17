#include "LinearRegressor.h"
#include <numbers>
#include <boost/math/distributions/students_t.hpp>

namespace alice_lri {

    LRResult LinearRegressor::fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const bool computeMse) {
        const double n = static_cast<double>(x.size());
        const double Sx = x.sum();
        const double Sy = y.sum();
        const double Sxx = x.square().sum();
        const double Sxy = (x * y).sum();

        const double Delta = n * Sxx - Sx * Sx;
        const double slope = (n * Sxy - Sx * Sy) / Delta;
        const double intercept = (Sxx * Sy - Sx * Sxy) / Delta;

        std::optional<double> mse;
        if (computeMse) {
            const auto yPred = slope * x + intercept;
            const auto residuals = y - yPred;
            mse = residuals.square().mean();
        }

        return LRResult(slope, intercept, mse);
    }

    WLSResult LinearRegressor::wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds) {
        const Eigen::ArrayXd &weights = 1 / bounds.square();

        const double S = weights.sum();
        const double Sx = (weights.array() * x.array()).sum();
        const double Sy = (weights.array() * y.array()).sum();
        const double Sxx = (weights.array() * x.array().square()).sum();
        const double Sxy = (weights.array() * x.array() * y.array()).sum();

        const double Delta = S * Sxx - Sx * Sx;
        const double slope = (S * Sxy - Sx * Sy) / Delta;
        const double intercept = (Sxx * Sy - Sx * Sxy) / Delta;

        const Eigen::VectorXd residuals = y.array() - (slope * x.array() + intercept);
        const double sigma2 = (weights.array() * residuals.array().square()).sum() / (static_cast<double>(y.size()) - 2);

        const double slopeVariance = sigma2 * S / Delta;
        const double interceptVariance = sigma2 * Sxx / Delta;
        const double ssr = (weights.array() * residuals.array().square()).sum();

        const double sizeOverTwo = static_cast<double>(y.size()) / 2;
        double logLikelihood = -std::log(ssr) * sizeOverTwo;
        logLikelihood -= (1 + std::log(std::numbers::pi / sizeOverTwo)) * sizeOverTwo;
        logLikelihood += 0.5 * weights.log().sum();

        const int df = static_cast<int>(y.size()) - 2;
        const boost::math::students_t dist(df);
        const double tCritical = quantile(complement(dist, 0.025)); // 95% CI

        Eigen::Array2d slopeCi;
        Eigen::Array2d interceptCi;

        for (int i = 0; i < 2; ++i) {
            Eigen::Array2d &currentCi = (i == 0) ? slopeCi : interceptCi;
            const double currentParameter = (i == 0) ? slope : intercept;
            const double currentVariance = (i == 0) ? slopeVariance : interceptVariance;

            const double standardError = std::sqrt(currentVariance);

            currentCi(0) = currentParameter - tCritical * standardError;
            currentCi(1) = currentParameter + tCritical * standardError;
        }

        return {
            .slope = slope,
            .intercept = intercept,
            .slopeVariance = slopeVariance,
            .interceptVariance = interceptVariance,
            .logLikelihood = logLikelihood,
            .slopeCi = std::move(slopeCi),
            .interceptCi = std::move(interceptCi)
        };
    }
}