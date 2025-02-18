#include "VerticalIntrinsicsEstimator.h"
#include <algorithm>
#include <boost/math/distributions/students_t.hpp>

#include "utils/Utils.h"

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

        Eigen::ArrayXi pointsScanlinesIds = Eigen::ArrayXi::Ones(points.size()) * -1;
        int64_t unassignedPoints = points.size();
        int64_t iteration = -1;
        uint64_t currentScanlineId = 0;

        double maxRange = points.getRanges().maxCoeff();
        double minRange = points.getRanges().minCoeff();

        while (unassignedPoints > 0) {
            iteration++;
            if (iteration > MAX_ITERATIONS) {
                break;
            }

            const std::optional<HoughCell> houghMaxOpt = hough->findMaximum(std::nullopt);

            if (!houghMaxOpt) {
                break;
            }

            const HoughCell houghMax = *houghMaxOpt;
            OffsetAngle maxValues = houghMax.maxValues;

            const OffsetAngleMargin margin = {
                {hough->getXStep(), hough->getXStep()},
                {hough->getYStep(), hough->getYStep()}
            };

            VerticalBounds errorBounds = computeErrorBounds(points, maxValues.offset);
            ScanlineLimits scanlineLimits = computeScanlineLimits(
                points, errorBounds.final, maxValues, margin, 0
            );

            bool requiresHeuristicFitting = true;
            bool fitSuccess = false;
            double uncertainty;
            OffsetAngleMargin confidenceIntervals = {};

            if (scanlineLimits.indices.size() > 2) {
                const ScanlineFitResult scanlineFit = tryFitScanline(
                    points, maxValues, errorBounds, scanlineLimits
                );

                requiresHeuristicFitting = scanlineFit.ciTooWide;
                fitSuccess = scanlineFit.success;

                if (fitSuccess and !requiresHeuristicFitting) {
                    // TODO these form a relevant group
                    maxValues = scanlineFit.fit->values;
                    scanlineLimits = *scanlineFit.limits;
                    confidenceIntervals = scanlineFit.fit->ci;
                    uncertainty = scanlineFit.fit->aic;
                    // TODO At this point, we know that the fit was successful, make the code more readable
                }
            }

            if (requiresHeuristicFitting and scanlineLimits.indices.size() > 0) {
                // TODO: in theory it can only ever be zero if it was before, perhaps break or something earlier
                const Eigen::ArrayXd invRanges = points.getInvRanges()(scanlineLimits.mask);
                const Eigen::ArrayXd phis = points.getPhis()(scanlineLimits.mask);

                double invRangesMean = invRanges.mean();
                double phisMean = phis.mean();

                HeuristicScanline heuristic = computeHeuristicScanline(invRangesMean, phisMean);
                maxValues.offset = heuristic.offset;
                maxValues.angle = (phis - (maxValues.offset * invRanges).asin()).mean();

                confidenceIntervals.offset = heuristic.offsetCi;

                RealMargin angleMarginTmp = {
                    (phis - (confidenceIntervals.offset.lower * invRanges).asin()).mean(),
                    (phis - (confidenceIntervals.offset.upper * invRanges).asin()).mean()
                };

                confidenceIntervals.angle = {
                    std::min(angleMarginTmp.lower, angleMarginTmp.upper),
                    std::max(angleMarginTmp.lower, angleMarginTmp.upper)
                };

                fitSuccess = true;
                uncertainty = std::numeric_limits<double>::infinity();

                double offsetMargin = confidenceIntervals.offset.diff() / 2;
                double angleMargin = confidenceIntervals.angle.diff() / 2;

                // Numerical stability
                offsetMargin = std::max(offsetMargin, 1e-6);
                angleMargin = std::max(angleMargin, 1e-6);

                errorBounds = computeErrorBounds(points, maxValues.offset);

                OffsetAngleMargin heuristicMargin = {offsetMargin, offsetMargin, angleMargin, angleMargin};
                scanlineLimits = computeScanlineLimits(
                    points, errorBounds.final, maxValues, heuristicMargin, invRangesMean
                );
            }

            if (!fitSuccess or scanlineLimits.indices.size() == 0) {
                hough->removeIdenticalCells(houghMax);
                continue;
            }

            bool intersectsOtherScanline = (pointsScanlinesIds(scanlineLimits.indices) != -1).any();
            ScanlineAngleBounds angleBounds = {
                .bottom = {
                    .lower = confidenceIntervals.angle.lower + asin(confidenceIntervals.offset.lower / maxRange),
                    .upper = confidenceIntervals.angle.lower + asin(confidenceIntervals.offset.lower / minRange)
                },
                .top = {
                    .lower = confidenceIntervals.angle.upper + asin(confidenceIntervals.offset.upper / maxRange),
                    .upper = confidenceIntervals.angle.upper + asin(confidenceIntervals.offset.upper / minRange)
                }
            };

            Eigen::ArrayX<bool> intersectsTheoreticalMask = Eigen::ArrayX<bool>::Constant(currentScanlineId + 1, true);

            std::vector boundsPointers = { &ScanlineAngleBounds::bottom, &ScanlineAngleBounds::top };
            std::vector boundCiPointers = { &RealMargin::lower, &RealMargin::upper };


            for (const auto thisBound : boundsPointers) {
                double thisLower = (angleBounds.*thisBound).lower;
                double thisUpper = (angleBounds.*thisBound).upper;

                for (const auto otherBound : boundsPointers) {
                    Eigen::ArrayXd maxTheoreticalSigns = Eigen::ArrayXd::Ones(currentScanlineId + 1);
                    Eigen::ArrayXd minTheoreticalSigns = Eigen::ArrayXd::Ones(currentScanlineId + 1);

                    for (uint32_t id = 0; id < currentScanlineId + 1; id++) {
                        if (!scanlineInfoMap.contains(id)) {
                            minTheoreticalSigns[id] = maxTheoreticalSigns[id] = 1;
                            continue;
                        }

                        double otherLower = (scanlineInfoMap[id].theoreticalAngleBounds.*otherBound).lower;
                        double otherUpper = (scanlineInfoMap[id].theoreticalAngleBounds.*otherBound).upper;

                        minTheoreticalSigns[id] = Utils::compare(thisLower, otherLower);
                        maxTheoreticalSigns[id] = Utils::compare(thisUpper, otherUpper);
                    }

                    intersectsTheoreticalMask = intersectsTheoreticalMask && ((maxTheoreticalSigns * minTheoreticalSigns).array() != 1).array();
                }
            }
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
    // TODO margins need to be arguments
    ScanlineLimits VerticalIntrinsicsEstimator::computeScanlineLimits(
        const PointArray &points, const Eigen::ArrayXd &errorBounds, const OffsetAngle &scanlineAttributes,
        const OffsetAngleMargin &margin, const double invRangesShift
    ) const {
        const auto &invRanges = points.getInvRanges();
        const auto &phis = points.getPhis();
        const auto offset = scanlineAttributes.offset;
        const auto angle = scanlineAttributes.angle;

        const Eigen::ArrayXd upperArcsinArg = (offset + margin.offset.upper) * invRanges.array().min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArg = (offset - margin.offset.lower) * invRanges.array().min(1).max(-1);

        const Eigen::ArrayXd upperArcsinArgShifted =
                (offset + margin.offset.upper) * (invRanges.array() - invRangesShift).min(1).max(-1);
        const Eigen::ArrayXd lowerArcsinArgShifted =
                (offset - margin.offset.lower) * (invRanges.array() - invRangesShift).min(1).max(-1);

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

        scanlineUpperLimit += deltaUpper + margin.angle.upper + errorBounds.array();
        scanlineLowerLimit += deltaLower - margin.angle.lower - errorBounds.array();

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

    ScanlineFitResult VerticalIntrinsicsEstimator::tryFitScanline(
        const PointArray &points, const OffsetAngle &scanlineAttributes, const VerticalBounds &errorBounds,
        const ScanlineLimits &scanlineLimits
    ) const {
        const Eigen::ArrayXd &ranges = points.getRanges().array();
        VerticalBounds currentErrorBounds = errorBounds;
        ScanlineLimits currentScanlineLimits = scanlineLimits;

        std::optional<LinearFitResult> fitResult = std::nullopt;
        std::optional<ScanlineLimits> limits = std::nullopt;
        FitConvergenceState state = INITIAL;
        bool ciTooWide = false;

        for (uint64_t attempt = 0; attempt < MAX_FIT_ATTEMPTS; attempt++) {
            const Eigen::ArrayXi &currentScanlineIndices = currentScanlineLimits.indices;

            if (currentScanlineIndices.size() <= 2) {
                break;
            }

            Eigen::ArrayX<bool> const *pointsToFitMask = &currentScanlineLimits.mask;
            Eigen::ArrayX<bool> newPointsToFitMask;

            if (state == INITIAL) {
                newPointsToFitMask = *pointsToFitMask && (ranges >= 2);
                pointsToFitMask = &newPointsToFitMask;

                if (pointsToFitMask->size() <= 2) {
                    pointsToFitMask = &currentScanlineLimits.mask;
                }
            }

            const Eigen::ArrayXd &invRanges = points.getInvRanges();
            const Eigen::ArrayXd &phis = points.getPhis();

            // TODO, careful, maybe this does not work. But: https://eigen.tuxfamily.org/dox/group__TutorialSlicingIndexing.html#title5
            // TODO check if we can avoid these copies by lazy evaluating inside the performFit function
            Eigen::ArrayXd invRangesFiltered = invRanges(*pointsToFitMask);
            Eigen::ArrayXd phisFiltered = phis(*pointsToFitMask);
            Eigen::ArrayXd boundsFiltered = currentErrorBounds.final(*pointsToFitMask);

            fitResult = performLinearFit(invRanges, phis, boundsFiltered);
            double offsetCiWidth = fitResult->ci.offset.upper - fitResult->ci.offset.lower;

            // TODO review and justify these constants
            if (offsetCiWidth > std::max(0.05 * fitResult->values.offset, 1e-2)) {
                ciTooWide = true;
                break;
            }

            // TODO we could save memory reallocation here (perhaps by passing buffer by ref to computeErrorBounds)
            currentErrorBounds = computeErrorBounds(points, fitResult->values.offset);

            // TODO take into account that strict fit was removed here
            double upperOffsetMargin = fitResult->ci.offset.upper - fitResult->values.offset;
            double lowerOffsetMargin = fitResult->values.offset - fitResult->ci.offset.lower;
            double upperAngleMargin = fitResult->ci.angle.upper - fitResult->values.angle;
            double lowerAngleMargin = fitResult->values.angle - fitResult->ci.angle.lower;

            // TODO 1e-6 can be justified for numerical stability, but well need to review 5e-4
            upperOffsetMargin = std::max(upperOffsetMargin, 5e-4);
            lowerOffsetMargin = std::max(lowerOffsetMargin, 5e-4);
            upperAngleMargin = std::max(upperAngleMargin, 1e-6);
            lowerAngleMargin = std::max(lowerAngleMargin, 1e-6);

            OffsetAngleMargin margin = {
                {lowerOffsetMargin, upperOffsetMargin},
                {lowerAngleMargin, upperAngleMargin}
            };
            const double meanInvRanges = invRangesFiltered.mean();
            ScanlineLimits newLimits = computeScanlineLimits(
                points, currentErrorBounds.final, fitResult->values, margin, meanInvRanges
            );

            // TODO perhaps this does not work as the sizes might be different, if so, use boolean masks instead
            if ((newLimits.indices == currentScanlineLimits.indices).all()) {
                if (state == CONVERGED) {
                    state = CONFIRMED;
                    break;
                }

                state = CONVERGED;
            }

            currentScanlineLimits = newLimits;
        }

        return {
            .success = state == CONFIRMED,
            .ciTooWide = ciTooWide,
            .fit = state == CONFIRMED ? fitResult : std::nullopt,
            .limits = state == CONFIRMED ? std::make_optional(currentScanlineLimits) : std::nullopt
        };
    }

    LinearFitResult VerticalIntrinsicsEstimator::performLinearFit(
        const Eigen::ArrayXd &invRanges, const Eigen::ArrayXd &phis, const Eigen::ArrayXd &bounds
    ) {
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

        int df = phis.size() - 2; // Degrees of freedom = n - k (k=2 for intercept & slope)
        boost::math::students_t dist(df);
        double tCritical = quantile(complement(dist, 0.025)); // 95% CI

        OffsetAngleMargin ci = {};
        for (int i = 0; i < 2; ++i) {
            const double standardError = std::sqrt(covariance(i, i)); // Standard error of beta[i]
            RealMargin &currentCi = (i == 0) ? ci.offset : ci.angle;

            currentCi.lower = beta[i] - tCritical * standardError;
            currentCi.upper = beta[i] + tCritical * standardError;
        }

        return {
            .values = {beta[0], beta[1]},
            .variance = {covariance(0, 0), covariance(1, 1)},
            .ci = ci,
            .aic = aic
        };
    }

    HeuristicScanline VerticalIntrinsicsEstimator::computeHeuristicScanline(
        double invRangesMean, double phisMean
    ) const {
        std::optional<uint32_t> closestScanlineIdTop = std::nullopt;
        std::optional<uint32_t> closestScanlineIdBottom = std::nullopt;
        double closestScanlineTopDistance = std::numeric_limits<double>::infinity();
        double closestScanlineBottomDistance = std::numeric_limits<double>::infinity();

        for (const auto &[scanlineId, scanline]: scanlineInfoMap) {
            const double scanlinePhi = std::asin(scanline.values.offset * invRangesMean) + scanline.values.angle;
            if (scanlinePhi > phisMean) {
                const double distance = scanlinePhi - phisMean;
                if (distance < closestScanlineTopDistance) {
                    closestScanlineIdTop = scanlineId;
                    closestScanlineTopDistance = distance;
                }
            }
            if (scanlinePhi < phisMean) {
                const double distance = phisMean - scanlinePhi;
                if (distance < closestScanlineBottomDistance) {
                    closestScanlineIdBottom = scanlineId;
                    closestScanlineBottomDistance = distance;
                }
            }
        }

        std::vector<uint32_t> validScanlineIds;
        if (closestScanlineIdTop) {
            validScanlineIds.emplace_back(*closestScanlineIdTop);
        }
        if (closestScanlineIdBottom) {
            validScanlineIds.emplace_back(*closestScanlineIdBottom);
        }

        assert(!validScanlineIds.empty() && "No valid scanlines found");

        std::vector<double> offsetDiffs;
        for (const auto &scanlineId: validScanlineIds) {
            const auto &scanline = scanlineInfoMap.at(scanlineId);
            double offsetDiff = scanline.ci.offset.upper - scanline.ci.offset.lower;
            offsetDiffs.emplace_back(offsetDiff);
        }

        const double maxOffsetDiff = std::ranges::max(offsetDiffs);
        double meanOffset = 0;
        for (const auto &scanlineId: validScanlineIds) {
            meanOffset += scanlineInfoMap.at(scanlineId).values.offset / static_cast<double>(validScanlineIds.size());
        }

        return HeuristicScanline{
            .offset = meanOffset,
            .offsetCi = {meanOffset - maxOffsetDiff / 2, meanOffset + maxOffsetDiff / 2},
            .dependencies = validScanlineIds
        };
    }
} // namespace accurate_ri
