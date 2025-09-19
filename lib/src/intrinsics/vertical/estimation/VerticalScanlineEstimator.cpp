#include "VerticalScanlineEstimator.h"

#include <BuildOptions.h>
#include <optional>

#include "Constants.h"
#include "intrinsics/vertical/estimation/VerticalHeuristicsEstimator.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "point/PointArray.h"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"
#include "utils/Utils.h"

namespace alice_lri {

    std::optional<VerticalScanlineEstimation> VerticalScanlineEstimator::estimate(
        const PointArray &points, const VerticalScanlinePool &scanlinePool,
        const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        if (scanlineLimits.indices.size() == 0) {
            return std::nullopt;
        }

        bool requiresHeuristics = false;
        if (scanlineLimits.indices.size() > 2) {
            ScanlineFitResult scanlineFit = tryFitScanline(points, scanlinePool, errorBounds, scanlineLimits);

            if (scanlineFit.success && scanlineFit.validCi) {
                return scanlineFitToEstimation(points, scanlineFit);
            }

            requiresHeuristics = !scanlineFit.validCi;
        }

        if constexpr (BuildOptions::USE_VERTICAL_HEURISTICS) {
            if (requiresHeuristics) {
                return VerticalHeuristicsEstimator::estimate(points, scanlinePool, scanlineLimits);
            }
        }

        return std::nullopt;
    }

    ScanlineFitResult VerticalScanlineEstimator::tryFitScanline(
        const PointArray &points, const VerticalScanlinePool &scanlinePool,
        const VerticalBounds &errorBounds, const ScanlineLimits &scanlineLimits
    ) {
        PROFILE_SCOPE("VerticalScanlineEstimator::tryFitScanline");
        VerticalBounds currentErrorBounds = errorBounds;
        ScanlineLimits currentScanlineLimits = scanlineLimits;

        std::optional<WLSResult> fitResult;
        auto convergenceState = FitConvergenceState::INITIAL;
        bool validCi = false;

        for (uint64_t attempt = 0; attempt < Constant::VERTICAL_MAX_FIT_ATTEMPTS; ++attempt) {
            const auto pointsToFitIndices = refinePointsToFitIndices(points, currentScanlineLimits, convergenceState);
            if (!pointsToFitIndices) {
                break;
            }

            fitResult = fitScanline(points, *pointsToFitIndices, currentErrorBounds, convergenceState);
            validCi = verifyConfidenceIntervals(*fitResult);
            if (!validCi) {
                break;
            }

            currentErrorBounds = VerticalScanlineLimits::computeErrorBounds(points, fitResult->slope);
            const ScanlineLimits newLimits = computeLimits(points, scanlinePool, *fitResult, currentErrorBounds);

            convergenceState = computeConvergenceState(currentScanlineLimits.mask, newLimits.mask, convergenceState);
            if (convergenceState == FitConvergenceState::CONFIRMED) {
                break;
            }

            currentScanlineLimits = newLimits;
        }

        return makeFitResult(currentScanlineLimits, fitResult, convergenceState, validCi);
    }

    std::optional<Eigen::ArrayXi> VerticalScanlineEstimator::refinePointsToFitIndices(
        const PointArray &points, const ScanlineLimits &scanlineLimits, const FitConvergenceState state
    ) {
        if (scanlineLimits.indices.size() <= 2) {
            return std::nullopt;
        }

        Eigen::ArrayX<bool> const *pointsToFitMask = &scanlineLimits.mask;
        Eigen::ArrayX<bool> newPointsToFitMask;

        if (state == FitConvergenceState::INITIAL) {
            newPointsToFitMask = *pointsToFitMask && (points.getRanges() >= 2);
            pointsToFitMask = &newPointsToFitMask;

            if (pointsToFitMask->count() <= 2) {
                pointsToFitMask = &scanlineLimits.mask;
            }
        }

        return Utils::eigenMaskToIndices(*pointsToFitMask);
    }

    WLSResult VerticalScanlineEstimator::fitScanline(
        const PointArray &points, const Eigen::ArrayXi &pointsToFitIndices, const VerticalBounds &errorBounds,
        const FitConvergenceState state
    ) {
        const Eigen::ArrayXd &invRanges = points.getInvRanges();
        const Eigen::ArrayXd &phis = points.getPhis();

        const Eigen::ArrayXd &invRangesFiltered = invRanges(pointsToFitIndices);
        const Eigen::ArrayXd &phisFiltered = phis(pointsToFitIndices);
        const Eigen::ArrayXd &boundsFiltered = errorBounds.final(pointsToFitIndices);

        WLSResult fitResult = LinearRegressor::wlsBoundsFit(invRangesFiltered, phisFiltered, boundsFiltered);
        int32_t pointFitCount = static_cast<int32_t>(pointsToFitIndices.size());

        LOG_INFO(
            "Model fit iteration; Offset: ", fitResult.slope, ", Angle: ", fitResult.intercept,
            " using ", pointFitCount, " points, Convergence state: ", static_cast<int>(state), ","
        );

        return fitResult;
    }

    bool VerticalScanlineEstimator::verifyConfidenceIntervals(const WLSResult &fitResult) {
        double offsetCiWidth = fitResult.slopeCi(1) - fitResult.slopeCi(0);
        if (offsetCiWidth > 1e-2) {
            ciTooWideState++;

            if (ciTooWideState >= 2) {
                LOG_INFO("CI too wide: ", offsetCiWidth);
                return false;
            }
        } else {
            ciTooWideState = 0;
        }

        return true;
    }

    ScanlineLimits VerticalScanlineEstimator::computeLimits(
        const PointArray& points, const VerticalScanlinePool &scanlinePool, const WLSResult &fitResult,
        const VerticalBounds &errorBounds
    ) {
        const VerticalMargin margin = scanlinePool.getHoughMargin();
        ScanlineLimits limits = VerticalScanlineLimits::computeScanlineLimits(
            points, errorBounds.final, fitResult.slope, fitResult.intercept, margin
        );

        return limits;
    }

    VerticalScanlineEstimator::FitConvergenceState VerticalScanlineEstimator::computeConvergenceState(
        const Eigen::ArrayX<bool> &oldMask, const Eigen::ArrayX<bool> &newMask, const FitConvergenceState oldState
    ) {
        if ((newMask == oldMask).all()) {
            if (oldState == FitConvergenceState::CONVERGED) {
                return FitConvergenceState::CONFIRMED;
            }

            return FitConvergenceState::CONVERGED;
        }

        return oldState;
    }

    ScanlineFitResult VerticalScanlineEstimator::makeFitResult(
        const ScanlineLimits &currentScanlineLimits, const std::optional<WLSResult> &fitResult,
        const FitConvergenceState convergenceState, bool validCi
    ) {
        return {
            .fit = convergenceState == FitConvergenceState::CONFIRMED ? fitResult : std::nullopt,
            .limits = convergenceState == FitConvergenceState::CONFIRMED
                          ? std::make_optional(currentScanlineLimits)
                          : std::nullopt,
            .success = convergenceState == FitConvergenceState::CONFIRMED,
            .validCi = validCi,
        };
    }

    std::optional<VerticalScanlineEstimation> VerticalScanlineEstimator::scanlineFitToEstimation(
        const PointArray &points, ScanlineFitResult& scanlineFit
    ) {
        ValueConfInterval offset = {
            .value = scanlineFit.fit->slope,
            .ci = { .lower = scanlineFit.fit->slopeCi[0], .upper = scanlineFit.fit->slopeCi[1] }
        };

        const ValueConfInterval angle = {
            .value = scanlineFit.fit->intercept,
            .ci = { .lower = scanlineFit.fit->interceptCi[0], .upper = scanlineFit.fit->interceptCi[1] }
        };

        offset.ci.clampBoth(-points.getMinRange(), points.getMinRange());

        return VerticalScanlineEstimation{
            .heuristic = false,
            .uncertainty = -scanlineFit.fit->logLikelihood,
            .offset = offset,
            .angle = angle,
            .limits = std::move(*scanlineFit.limits)
        };
    }
}
