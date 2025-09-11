#include "VerticalIntrinsicsEstimator.h"
#include <algorithm>
#include <queue>
#include <ranges>
#include "helper/VerticalLogging.h"
#include "utils/Utils.h"
#include "utils/TestUtils.h"
#include "utils/Timer.h"
#include "Constants.h"
#include "BuildOptions.h"
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineEstimator.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"

namespace accurate_ri {
    VerticalScanlinePool VerticalIntrinsicsEstimator::init(const PointArray &points) {
        const double offsetMax = std::min(points.getRanges().minCoeff(), Constant::MAX_OFFSET) - Constant::OFFSET_STEP;
        const double offsetMin = -offsetMax;

        constexpr double angleMax = M_PI / 2 - Constant::ANGLE_STEP;
        constexpr double angleMin = -angleMax;

        VerticalScanlinePool scanlinePool(
            offsetMin, offsetMax, Constant::OFFSET_STEP, angleMin, angleMax, Constant::ANGLE_STEP
        );

        VerticalLogging::printHeaderDebugInfo(points, scanlinePool);
        scanlinePool.performPrecomputations(points);

        return scanlinePool;
    }

    // TODO perhaps further split this function, perhaps when we construct the result iteratively
    VerticalIntrinsicsEstimation VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        VerticalScanlinePool scanlinePool = init(points);
        ScanlineConflictSolver conflictSolver;

        int64_t iteration = -1;
        uint32_t currentScanlineId = 0;
        EndReason endReason = EndReason::ALL_ASSIGNED;

        while (scanlinePool.anyUnassigned()) {
            iteration++;

            VerticalScanlineHoughCandidate houghCandidate = findCandidate(scanlinePool, iteration);
            if (!houghCandidate.available) {
                endReason = *houghCandidate.endReason;
                break;
            }

            const auto estimation = estimateScanline(points, scanlinePool, *houghCandidate.estimation);
            if (!estimation) {
                scanlinePool.invalidateByHash(houghCandidate.estimation->cell.hash);
                continue;
            }

            const auto angleBounds = estimation->toAngleBounds(points.getMinRange(), points.getMaxRange());
            const auto candidate = VerticalScanlineCandidate{
                .scanline = makeVerticalScanline(currentScanlineId, houghCandidate, estimation, angleBounds),
                .limits = estimation->limits
            };

            bool keepScanline;
            if constexpr (BuildOptions::USE_SCANLINE_CONFLICT_SOLVER) {
                keepScanline = conflictSolver.performScanlineConflictResolution(scanlinePool, points, candidate);
            } else {
                keepScanline = ScanlineConflictSolver::simpleShouldKeep(scanlinePool, candidate);
            }

            if (!keepScanline) {
                continue;
            }

            scanlinePool.acceptCandidate(points, candidate);
            VerticalLogging::logScanlineAssignation(candidate.scanline);
            LOG_INFO("Number of unassigned points: ", scanlinePool.getUnassignedPoints());
            LOG_INFO("");

            currentScanlineId++;
        }

        const VerticalIntrinsicsEstimation result = extractResult(iteration, points, scanlinePool, endReason);
        return result;
    }

    VerticalScanlineHoughCandidate VerticalIntrinsicsEstimator::findCandidate(
        const VerticalScanlinePool &scanlinePool, const int64_t iteration
    ) {
        VerticalScanlineHoughCandidate result;

        if (iteration > Constant::VERTICAL_MAX_ITERATIONS) {
            LOG_WARN("Warning: Maximum iterations reached");
            result.endReason = EndReason::MAX_ITERATIONS;
            return result;
        }

        result.estimation = scanlinePool.performHoughEstimation();

        if (!result.estimation) {
            LOG_WARN("Warning: No more peaks found");
            result.endReason = EndReason::NO_MORE_PEAKS;
            return result;
        }

        result.available = true;
        VerticalLogging::logHoughInfo(iteration, result.estimation->cell);

        return result;
    }

    std::optional<VerticalScanlineEstimation> VerticalIntrinsicsEstimator::estimateScanline(
        const PointArray &points, const VerticalScanlinePool& scanlinePool, const HoughScanlineEstimation &hough
    ) {
        const VerticalMargin &margin = hough.margin;
        const HoughCell &houghMax = hough.cell;

        const VerticalBounds errorBounds = VerticalScanlineLimits::computeErrorBounds(points, houghMax.maxOffset);
        const ScanlineLimits scanlineLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, errorBounds.final, houghMax.maxOffset, houghMax.maxAngle, margin
        );

        VerticalScanlineEstimator scanlineEstimator;
        std::optional<VerticalScanlineEstimation> estimationResultOpt = scanlineEstimator.estimate(
            points, scanlinePool, errorBounds, scanlineLimits
        );

        if (!estimationResultOpt) {
            LOG_INFO("Fit failed: True, Points in scanline: ", scanlineLimits.indices.size());
            LOG_INFO("");

            return std::nullopt;
        }

        return estimationResultOpt;
    }

    VerticalIntrinsicsEstimation VerticalIntrinsicsEstimator::extractResult(
        const int64_t iteration, const PointArray &points, VerticalScanlinePool& scanlinePool,
        const EndReason endReason
    ) {
        int64_t unassignedPoints = scanlinePool.getUnassignedPoints();

        if (scanlinePool.anyUnassigned()) {
            LOG_WARN("Warning: Found ", unassignedPoints, " spurious points");
        }

        VerticalScanlinesAssignations scanlinesAssignations = scanlinePool.extractFullSortedScanlineAssignations();

        LOG_INFO("Number of scanlines: ", scanlinesAssignations.scanlines.size());
        LOG_INFO("Number of unassigned points: ", unassignedPoints);

        return VerticalIntrinsicsEstimation{
            .iterations = static_cast<int32_t>(iteration),
            .unassignedPoints = static_cast<int32_t>(unassignedPoints),
            .pointsCount = static_cast<int32_t>(points.size()),
            .endReason = endReason,
            .scanlinesAssignations = std::move(scanlinesAssignations)
        };
    }

    VerticalScanline VerticalIntrinsicsEstimator::makeVerticalScanline(
        const uint32_t currentScanlineId, const VerticalScanlineHoughCandidate &houghCandidate,
        const std::optional<VerticalScanlineEstimation> &estimation, const ScanlineAngleBounds &angleBounds
    ) {
        return VerticalScanline{
            .id = currentScanlineId,
            .pointsCount = static_cast<uint64_t>(estimation->limits.indices.size()),
            .angle = estimation->angle,
            .offset = estimation->offset,
            .theoreticalAngleBounds = angleBounds,
            .uncertainty = estimation->uncertainty,
            .hough = *houghCandidate.estimation,
        };
    }
} // namespace accurate_ri
