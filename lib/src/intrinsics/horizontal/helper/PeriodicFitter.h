#pragma once
#include "math/LinearRegressor.h"
#include "math/Stats.h"


namespace accurate_ri {
    class PeriodicFitter {
    public:
        static LRResult fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, double period, double slopeGuess);

    private:
        struct NewMultiLineResult {
            Eigen::ArrayXd residuals;
            Eigen::ArrayXi linesIdx;
        };

        static void computePeriodicResiduals(
            const Eigen::ArrayXd& x, const Eigen::ArrayXd& y, double period, double slope,
            double intercept, NewMultiLineResult& outResult
        );

        static double computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, double period);

        static LRResult refineFit(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, NewMultiLineResult &multiLineResult, double period
        );
    };
}
