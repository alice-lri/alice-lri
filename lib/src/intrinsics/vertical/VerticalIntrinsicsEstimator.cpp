#include "VerticalIntrinsicsEstimator.h"
#include <algorithm>
#include <boost/math/distributions/students_t.hpp>

namespace accurate_ri {
    // TODO extract these constants somewhere
    constexpr uint64_t MAX_ITERATIONS = 10000;
    constexpr double MAX_OFFSET = 0.5;
    constexpr double OFFSET_STEP = 1e-3;
    constexpr double ANGLE_STEP = 1e-4;
    constexpr uint64_t MAX_FIT_ATTEMPTS = 10;

    enum FitConvergenceState {
        INITIAL,
        CONVERGED,
        CONFIRMED,
    };

    // TODO probably make dedicated structs for all the tuples
    void VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        initHough(points);

        hough->computeAccumulator(points);

        int64_t unassignedPoints = points.size();
        int64_t iteration = -1;

        while (unassignedPoints > 0) {
            iteration++;
            if (iteration > MAX_ITERATIONS) {
                break;
            }

            const std::optional<HoughCell> houghMaxOpt = hough->findMaximum(std::nullopt);

            if (!houghMaxOpt) {
                break;
            }

            const auto houghMax = *houghMaxOpt;
            const auto errorBounds = computeErrorBounds(points, houghMax.maxValues.offset);
            const auto scanlineLimits = computeScanlineLimits(points, errorBounds.final, houghMax.maxValues, 0);
        }
    }

    void VerticalIntrinsicsEstimator::initHough(const PointArray &points) {
        double offsetMax = std::min(std::ranges::min(points.getRanges()), MAX_OFFSET) - OFFSET_STEP;
        double offsetMin = -offsetMax;

        double angleMax = M_PI / 2 - ANGLE_STEP;
        double angleMin = -angleMax;

        hough = std::make_unique<HoughTransform>(offsetMin, offsetMax, OFFSET_STEP, angleMin, angleMax, ANGLE_STEP);
    }

    // TODO precompute on PointArray
    // TODO review equation and make code cleaner
    VerticalBounds VerticalIntrinsicsEstimator::computeErrorBounds(
        const PointArray &points, const double offset
    ) {
        double coordsEps = points.getCoordsEps();
        const auto &zs = points.getZ();
        const auto &rangesXy = points.getRangesXy();
        const auto &ranges = points.getRanges();

        const Eigen::ArrayXd rangeXySquared = rangesXy.array().square();
        const Eigen::ArrayXd rangeSquared = ranges.array().square();
        const Eigen::ArrayXd zsOverRangesXy = zs.array() / rangesXy.array();
        const Eigen::ArrayXd sqrtFactor = 1 + zsOverRangesXy.square();

        const Eigen::ArrayXd phisUpperBound = (coordsEps * std::sqrt(2) * zs.cwiseAbs() + coordsEps * rangesXy).array()
                                              / (rangeXySquared.array() * sqrtFactor.array());

        const Eigen::ArrayXd correctionUpperBound = offset * coordsEps * std::sqrt(3)
                                                    / (rangeSquared.array() * (1 - (offset / ranges.array()).square()).
                                                       sqrt());

        const Eigen::ArrayXd finalUpperBound = phisUpperBound + correctionUpperBound;


        return {phisUpperBound, correctionUpperBound, finalUpperBound};
    }

    // TODO split this function
    ScanlineLimits VerticalIntrinsicsEstimator::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const double invRangesShift
    ) const {
        const auto &invRanges = points.getInvRanges();
        const auto &phis = points.getPhis();
        const auto upperOffsetMargin = hough->getXStep();
        const auto upperAngleMargin = hough->getYStep();
        const auto lowerOffsetMargin = upperOffsetMargin;
        const auto lowerAngleMargin = upperAngleMargin;
        const auto offset = scanlineAttributes.offset;
        const auto angle = scanlineAttributes.angle;

        const Eigen::ArrayXd upperArcsinArg = (offset + upperOffsetMargin) * invRanges.array().min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArg = (offset - lowerOffsetMargin) * invRanges.array().min(1).max(-1);

        const Eigen::ArrayXd upperArcsinArgShifted =
                (offset + upperOffsetMargin) * (invRanges.array() - invRangesShift).min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArgShifted =
                (offset - lowerOffsetMargin) * (invRanges.array() - invRangesShift).min(1).max(-1);

        const Eigen::ArrayXd upperArcsin = upperArcsinArg.array().asin();
        const Eigen::ArrayXd lowerArcsin = lowerArcsinArg.array().asin();

        const Eigen::ArrayXd upperArcsinShifted = upperArcsinArgShifted.array().asin();
        const Eigen::ArrayXd lowerArcsinShifted = lowerArcsinArgShifted.array().asin();

        const Eigen::ArrayXd deltaUpper = upperArcsin - upperArcsinShifted;
        const Eigen::ArrayXd deltaLower = lowerArcsin - lowerArcsinShifted;

        const Eigen::ArrayXd scanlineUpperLimitTmp = upperArcsinShifted + angle;
        const Eigen::ArrayXd scanlineLowerLimitTmp = lowerArcsinShifted + angle;

        Eigen::ArrayXd scanlineUpperLimit = scanlineUpperLimitTmp.array().max(scanlineLowerLimitTmp.array());
        Eigen::ArrayXd scanlineLowerLimit = scanlineLowerLimitTmp.array().min(scanlineUpperLimitTmp.array());

        scanlineUpperLimit += deltaUpper + upperAngleMargin + errorBounds.array();
        scanlineLowerLimit += deltaLower - lowerAngleMargin - errorBounds.array();

        auto scanlineIndices = Eigen::ArrayXi(points.size());
        Eigen::ArrayX<bool> mask = scanlineLowerLimit.array() <= phis.array()
                                   && phis.array() <= scanlineUpperLimit.array();

        int32_t count = 0;
        for (int32_t i = 0; i < points.size(); i++) {
            if (mask[i]) {
                scanlineIndices[count++] = i;
            }
        }

        scanlineIndices.conservativeResize(count);


        return {scanlineIndices, mask, scanlineLowerLimit, scanlineUpperLimit};
    }

    void VerticalIntrinsicsEstimator::tryFitScanline(
        const PointArray &points, const OffsetAngle &scanlineAttributes, const VerticalBounds &errorBounds,
        const ScanlineLimits &scanlineLimits
    ) const {
        const Eigen::ArrayXi &currentScanlineIndices = scanlineLimits.indices;
        const Eigen::ArrayXd &ranges = points.getRanges().array();
        FitConvergenceState state = INITIAL;

        for (uint64_t attempt = 0; attempt < MAX_FIT_ATTEMPTS; attempt++) {
            if (currentScanlineIndices.size() <= 2) {
                break;
            }

            Eigen::ArrayX<bool> const *pointsToFitMask = &scanlineLimits.mask;
            Eigen::ArrayX<bool> newPointsToFitMask;

            if (state == INITIAL) {
                newPointsToFitMask = *pointsToFitMask && (ranges >= 2);
                pointsToFitMask = &newPointsToFitMask;

                if (pointsToFitMask->size() <= 2) {
                    pointsToFitMask = &scanlineLimits.mask;
                }
            }
        }
    }

    LinearFitResult performLinearFit(const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &bounds) {
        Eigen::ArrayXd weights = 1 / bounds.square();

        // Weighted least squares, variables use matrix notation from the standard formula, invRanges is X, phis is y
        Eigen::MatrixXd X = Eigen::MatrixXd(phis.size(), 2);
        X.col(0) = Eigen::VectorXd::Ones(phis.size());
        X.col(1) = invRanges;

        Eigen::MatrixXd W = weights.matrix().asDiagonal();
        Eigen::MatrixXd XtW = X.transpose() * W;
        Eigen::MatrixXd XtWX = XtW * X;
        Eigen::MatrixXd XtWy = XtW * phis.matrix();

        Eigen::VectorXd beta = XtWX.ldlt().solve(XtWy);
        Eigen::VectorXd residuals = phis.matrix() - X * beta;
        double sigma2 = (residuals.transpose() * W * residuals).value() / (phis.size() - 2);

        Eigen::MatrixXd covariance = XtWX.inverse() * sigma2;
        double log_likelihood = -0.5 * (phis.size() * std::log(2 * M_PI * sigma2) +
                                        (residuals.transpose() * W * residuals).sum() / sigma2);
        double aic = -2 * log_likelihood + 2 * 2; // 2 parameters

        int df = phis.size() - 2;  // Degrees of freedom = n - k (k=2 for intercept & slope)
        boost::math::students_t dist(df);
        double tCritical = quantile(complement(dist, 0.025));  // 95% CI

        OffsetAngleCI ci = {};
        for (int i = 0; i < 2; ++i) {
            const double standardError = std::sqrt(covariance(i, i));  // Standard error of beta[i]
            ConfidenceInterval &currentCi = (i == 0) ? ci.offset : ci.angle;

            currentCi.lower = beta[i] - tCritical * standardError;
            currentCi.upper = beta[i] + tCritical * standardError;
        }

        return {
            { beta[0], beta[1]},
            { covariance(0, 0), covariance(1, 1) },
            ci,
            aic
        };
    }
} // namespace accurate_ri
