#pragma once
#include "math/Stats.h"


namespace accurate_ri {
    class PeriodicFit {
    public:
        static Stats::LRResult fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, double period, double slopeGuess);

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

        static Stats::LRResult refineFit(
            const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, NewMultiLineResult &multiLineResult, double period
        );
    };
}
