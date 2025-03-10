#include <boost/math/distributions/students_t.hpp>
#include <eigen3/Eigen/Dense>
#include "Stats.h"
#include "utils/Timer.h"

namespace accurate_ri::Stats {
    WLSResult wlsBoundsFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &bounds) {
        PROFILE_SCOPE("Stats::wlsBoundsFit");
        const Eigen::ArrayXd &weights = 1 / bounds.square();

        const double S = weights.sum();
        const double Sx = (weights.array() * x.array()).sum();
        const double Sy = (weights.array() * y.array()).sum();
        const double Sxx = (weights.array() * x.array().square()).sum();
        const double Sxy = (weights.array() * x.array() * y.array()).sum();

        const double Delta = S * Sxx - Sx * Sx;
        const double slope = (S * Sxy - Sx * Sy) / Delta;
        const double intercept = (Sxx * Sy - Sx * Sxy) / Delta;

        // Compute residuals and weighted variance
        const Eigen::VectorXd residuals = y.array() - (slope * x.array() + intercept);
        const double sigma2 = (weights.array() * residuals.array().square()).sum() / (y.size() - 2);

        // Optionally, compute the covariance matrix explicitly
        const double slopeVariance = sigma2 * S / Delta;
        const double interceptVariance = sigma2 * Sxx / Delta;
        const double ssr = (weights.array() * residuals.array().square()).sum();

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
            Eigen::Array2d &currentCi = (i == 0) ? slopeCi : interceptCi;
            const double currentParameter = (i == 0) ? slope : intercept;
            const double currentVariance = (i == 0) ? slopeVariance : interceptVariance;

            const double standardError = std::sqrt(currentVariance); // Standard error of beta[i]

            currentCi(0) = currentParameter - tCritical * standardError;
            currentCi(1) = currentParameter + tCritical * standardError;
        }

        return {
            .slope = slope,
            .intercept = intercept,
            .slopeVariance = slopeVariance,
            .interceptVariance = interceptVariance,
            .aic = aic,
            .slopeCi = std::move(slopeCi),
            .interceptCi = std::move(interceptCi)
        };
    }

    LRResult simpleLinearRegression(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        const double n = static_cast<double>(x.size());
        const double Sx = x.sum();
        const double Sy = y.sum();
        const double Sxx = x.square().sum();
        const double Sxy = (x * y).sum();

        const double Delta = n * Sxx - Sx * Sx;
        const double slope = (n * Sxy - Sx * Sy) / Delta;
        const double intercept = (Sxx * Sy - Sx * Sxy) / Delta;

        return {
            .slope = slope,
            .intercept = intercept
        };
    }

    template<typename T>
    T intModeImpl(const std::vector<T> &values) {
        if (values.empty()) {
            return 0;
        }

        std::unordered_map<T, int64_t> frequencies;

        for (const auto &value : values) {
            frequencies[value]++;
        }

        T mode = values[0];
        T maxCount = 0;

        for (const auto &[value, count] : frequencies) {
            if (count > maxCount) {
                maxCount = count;
                mode = value;
            }
        }

        return mode;
    }

    int64_t intMode(const std::vector<int64_t> &values) {
        return intModeImpl(values);
    }

    int32_t intMode(const std::vector<int32_t> &values) {
        return intModeImpl(values);
    }
}
