#include <boost/math/distributions/students_t.hpp>
#include <eigen3/Eigen/Dense>
#include "Stats.h"
#include "utils/Timer.h"

namespace accurate_ri::Stats {
    // TODO the memory requirements explode with the number of points, check if this also happens in python
    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds) {
        PROFILE_SCOPE("Stats::wlsBoundsFit");
        const Eigen::ArrayXd &weights = 1 / bounds.square();

        // Weighted least squares, variables use matrix notation from the standard formula, invRanges is X, phis is y
        Eigen::MatrixXd X = Eigen::MatrixXd(y.size(), 2);
        X.col(1) = Eigen::VectorXd::Ones(y.size());
        X.col(0) = x;

        const Eigen::MatrixXd &W = weights.matrix().asDiagonal();
        const Eigen::MatrixXd &XtW = X.transpose() * W;
        const Eigen::MatrixXd &XtWX = XtW * X;
        const Eigen::MatrixXd &XtWy = XtW * y.matrix();

        const Eigen::VectorXd &beta = XtWX.ldlt().solve(XtWy);
        const Eigen::VectorXd &residuals = y.matrix() - X * beta;
        const double sigma2 = (residuals.transpose() * W * residuals).value() / (y.size() - 2);

        const Eigen::MatrixXd &covariance = XtWX.inverse() * sigma2;

        const double ssr = (residuals.transpose() * W * residuals).value();
        const double sizeOverTwo = static_cast<double>(y.size()) / 2;
        double logLikelihood = -std::log(ssr) * sizeOverTwo;
        logLikelihood -= (1 + std::log(M_PI / sizeOverTwo)) * sizeOverTwo;
        logLikelihood += 0.5 * weights.log().sum();
        const double aic = -2 * logLikelihood + 2 * 2; // 2 parameters

        const int df = y.size() - 2; // Degrees of freedom = n - k (k=2 for intercept & slope)
        boost::math::students_t dist(df);
        const double tCritical = quantile(complement(dist, 0.025)); // 95% CI

        Eigen::Array2d slopeCi;
        Eigen::Array2d interceptCi;

        for (int i = 0; i < 2; ++i) {
            const double standardError = std::sqrt(covariance(i, i)); // Standard error of beta[i]
            Eigen::Array2d &currentCi = (i == 0) ? slopeCi : interceptCi;

            currentCi(0) = beta[i] - tCritical * standardError;
            currentCi(1) = beta[i] + tCritical * standardError;
        }

        return {
            .slope = beta[0],
            .intercept = beta[1],
            .slopeVariance = covariance(0, 0),
            .interceptVariance = covariance(1, 1),
            .aic = aic,
            .slopeCi = std::move(slopeCi),
            .interceptCi = std::move(interceptCi)
        };
    }
}
