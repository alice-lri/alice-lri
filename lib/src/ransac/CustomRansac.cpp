#include "CustomRansac.h"
#include <limits>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include <vector>
#include "OsqpEigen/Solver.hpp"
#include "plotty/matplotlibcpp.hpp"

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

    uint32_t my_random() {
        static uint32_t state = 42;
        static const uint32_t a = 1664525;
        static const uint32_t c = 1013904223;
        state = a * state + c; // wraps automatically on 32-bit unsigned
        return state;
    }

    std::optional<CustomRansacResult> CustomRansac::fit(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx, const std::optional<double> offsetGuess
    ) {
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &x = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd y = HorizontalMath::computeDiffToIdeal(thetas, resolution, false); // TODO we are now recomputing this multiple times

        if (!offsetGuess) {
            const uint32_t sampleIndex1 = my_random() % y.size();
            uint32_t sampleIndex2 = my_random() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size();

            LOG_DEBUG("Selected indices: ", sampleIndex1, ", ", sampleIndex2);
            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            model = estimator.fit(sampleX, sampleY);
        } else {
            model->slope = offsetGuess.value();
            model->intercept = 0;
            estimator.setModel(*model);

            model->intercept = computeCircularMeanIntercept(estimator.computeResiduals(x, y), 2 * M_PI / resolution);
            estimator.setModel(*model);
        }

        LOG_DEBUG("Two-point slope: ", model->slope);
        const double loss = refineFit(x, y);
        return std::make_optional<CustomRansacResult>({.model = *model, .loss = loss});
    }

    double CustomRansac::refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        // TODO this is done to update the multilineresult, but maybe it can be avoided
        estimator.computeResiduals(x, y);
        const MultiLineResult &multi = estimator.getLastMultiLine();

        const double thetaStep = 2 * M_PI / resolution;
        const Eigen::ArrayXd shiftedY = y - multi.linesIdx.cast<double>() * thetaStep;

        // TODO now here we could use the wls fit method and the uncertainty maybe
        const Stats::LRResult fitResult = Stats::simpleLinearRegression(x, shiftedY, true);

        return *fitResult.mse;
    }

    // TODO fix this method code aesthetics
    bool CustomRansac::fitToBoundsQp(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &eps, const double slope,
        const double intercept, double &deltaSlopeOut, double &deltaInterceptOut
    ) {
        int n = x.size();
        Eigen::MatrixXd A(2 * n, 2);
        Eigen::VectorXd l(2 * n), u(2 * n);

        for (int i = 0; i < n; ++i) {
            double xi = x[i];
            double yi = y[i];
            double eps_i = eps[i];
            double predicted = slope * xi + intercept;

            // + constraint
            A.row(i) = Eigen::RowVector2d(xi, 1.0);
            l[i] = yi - eps_i - predicted;
            u[i] = yi + eps_i - predicted;

            // - constraint
            A.row(n + i) = -A.row(i);
            l[n + i] = -u[i];
            u[n + i] = -l[i];
        }

        // QP: min 1/2 xᵀPx + qᵀx
        Eigen::SparseMatrix<double> P(2, 2);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        Eigen::VectorXd q = Eigen::Vector2d::Zero();

        P.insert(0, 0) = 2.0;
        P.insert(1, 1) = 2.0;

        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.settings()->setAbsoluteTolerance(1e-8);
        solver.settings()->setRelativeTolerance(1e-8);

        solver.data()->setNumberOfVariables(2);
        solver.data()->setNumberOfConstraints(2 * n);

        if (!solver.data()->setHessianMatrix(P))
            return false;
        if (!solver.data()->setGradient(q))
            return false;
        if (!solver.data()->setLinearConstraintsMatrix(A_sparse))
            return false;
        if (!solver.data()->setLowerBound(l))
            return false;
        if (!solver.data()->setUpperBound(u))
            return false;

        if (!solver.initSolver())
            return false;

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return false;

        if (solver.getStatus() != OsqpEigen::Status::Solved)
            return false;

        Eigen::VectorXd solution = solver.getSolution();
        deltaSlopeOut = solution[0];
        deltaInterceptOut = solution[1];

        if (!std::isfinite(deltaSlopeOut)) {
            LOG_DEBUG("QP solution is invalid, infinite values");
            return false;
        }

        // Verify constraints
        bool all_satisfied = true;

        for (int i = 0; i < n; ++i) {
            double predicted_adjusted = (slope + deltaSlopeOut) * x[i] + (intercept + deltaInterceptOut);
            double residual = y[i] - predicted_adjusted;
            double max_error = eps[i];

            if (std::abs(residual) > max_error + 1e-8) {  // small tolerance for float error
                all_satisfied = false;
                break;
            }
        }

        if (!all_satisfied) {
            LOG_ERROR("QP solution is invalid, some constraints are violated");
            return false;
        }


        LOG_DEBUG("Bounds feasible. Delta slope: ", deltaSlopeOut);
        return true;
    }

    double CustomRansac::computeCircularMeanIntercept(const Eigen::ArrayXd& residuals, const double k) {
        constexpr double twoPi = 2.0 * M_PI;

        // Step 1: Map residuals mod k into [0, k)
        Eigen::ArrayXd mod = residuals.unaryExpr([k](double r) {
            double m = std::fmod(r, k);
            return m < 0 ? m + k : m;
        });

        // Step 2: Convert to angles in [0, 2π)
        Eigen::ArrayXd theta = (twoPi / k) * mod;

        // Step 3: Compute mean of cos and sin
        double x_sum = theta.cos().mean();
        double y_sum = theta.sin().mean();

        // Step 4: Compute circular mean angle
        double theta_bar = std::atan2(y_sum, x_sum);
        if (theta_bar < 0) theta_bar += twoPi;

        // Step 5: Convert back to b in [0, k)
        return (k * theta_bar) / twoPi;
    }
} // accurate_ri
