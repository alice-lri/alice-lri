#include "VerticalIntrinsicsEstimator.h"
#include <algorithm>
#include <queue>
#include <ranges>
#include "helper/VerticalLogging.h"
#include "utils/Utils.h"
#include "utils/TestUtils.h"
#include "utils/Timer.h"
#include <nlohmann/json.hpp>
#include "Constants.h"
#include "BuildOptions.h"
#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "intrinsics/vertical/estimation/VerticalScanlineEstimator.h"
#include "intrinsics/vertical/estimation/VerticalScanlineLimits.h"

namespace accurate_ri {
    void VerticalIntrinsicsEstimator::init(const PointArray &points) {
        double offsetMax = std::min(std::ranges::min(points.getRanges()), Constant::MAX_OFFSET) - Constant::OFFSET_STEP;
        double offsetMin = -offsetMax;

        double angleMax = M_PI / 2 - Constant::ANGLE_STEP;
        double angleMin = -angleMax;

        scanlinePool = std::make_unique<VerticalScanlinePool>(
            offsetMin, offsetMax, Constant::OFFSET_STEP, angleMin, angleMax, Constant::ANGLE_STEP
        );

        VerticalLogging::printHeaderDebugInfo(points, *scanlinePool);
        scanlinePool->performPrecomputations(points);
    }

    // TODO perhaps further split this function, perhaps when we construct the result iteratively
    VerticalIntrinsicsEstimation VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
        init(points);

        int64_t iteration = -1;
        uint32_t currentScanlineId = 0;
        EndReason endReason = EndReason::ALL_ASSIGNED;

        while (scanlinePool->anyUnassigned()) {
            iteration++;

            Candidate candidate = findCandidate(iteration);
            if (!candidate.valid) { // TODO valid is unintituive since we break and finish
                endReason = *candidate.endReason;
                break;
            }

            const std::optional<RefinedCandidate> refinedCandidate = refineCandidate(iteration, points, candidate);
            if (!refinedCandidate) {
                scanlinePool->invalidateByHash(candidate.hough->cell.hash);
                continue;
            }

            const ScanlineAngleBounds angleBounds = refinedCandidate->scanline
                    .toAngleBounds(points.getMinRange(), points.getMaxRange());

            bool keepScanline;
            if constexpr (BuildOptions::USE_SCANLINE_CONFLICT_SOLVER) {
                keepScanline = conflictSolver.performScanlineConflictResolution(
                    *scanlinePool, points, angleBounds, refinedCandidate->scanline, currentScanlineId,
                    candidate.hough->cell
                );
            } else {
                keepScanline = conflictSolver.simpleShouldKeep(
                    *scanlinePool, angleBounds, refinedCandidate->scanline, currentScanlineId, candidate.hough->cell
                );
            }

            if (!keepScanline) {
                continue;
            }

            const VerticalScanline scanline =
                makeVerticalScanline(currentScanlineId, candidate, refinedCandidate, angleBounds);

            // TODO merge?
            scanlinePool->removeVotes(points, refinedCandidate->scanline.limits.indices);
            scanlinePool->assignScanline(scanline, refinedCandidate->scanline.limits.indices);

            VerticalLogging::logScanlineAssignation(scanline);
            LOG_INFO("Number of unassigned points: ", scanlinePool->getUnassignedPoints());
            LOG_INFO("");

            currentScanlineId++;
        }

        const VerticalIntrinsicsEstimation result = extractResult(iteration, points, endReason);
        return result;
    }

    Candidate VerticalIntrinsicsEstimator::findCandidate(const int64_t iteration) const {
        Candidate result;

        if (iteration > Constant::VERTICAL_MAX_ITERATIONS) {
            LOG_WARN("Warning: Maximum iterations reached");
            result.endReason = EndReason::MAX_ITERATIONS;
            return result;
        }

        result.hough = scanlinePool->performHoughEstimation();

        if (!result.hough) {
            LOG_WARN("Warning: No more peaks found");
            result.endReason = EndReason::NO_MORE_PEAKS;
            return result;
        }

        result.valid = true;
        VerticalLogging::logHoughInfo(iteration, result.hough->cell);

        return result;
    }

    std::optional<RefinedCandidate> VerticalIntrinsicsEstimator::refineCandidate(
        const int64_t iteration, const PointArray &points, const Candidate &candidate
    ) const {
        const VerticalMargin &margin = candidate.hough->margin;
        const HoughCell &houghMax = candidate.hough->cell;

        VerticalBounds errorBounds = VerticalScanlineLimits::computeErrorBounds(points, houghMax.maxOffset);
        ScanlineLimits scanlineLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, errorBounds.final, houghMax.maxOffset, houghMax.maxAngle, margin
        );

        LOG_INFO("Minimum limit width (Hough): ", (scanlineLimits.upperLimit - scanlineLimits.lowerLimit).minCoeff());

        VerticalScanlineEstimator scanlineEstimator;
        std::optional<ScanlineEstimationResult> estimationResultOpt = scanlineEstimator.estimate(
            points, *scanlinePool, errorBounds, scanlineLimits
        );

        if (!estimationResultOpt) {
            LOG_INFO("Fit failed: True, Points in scanline: ", scanlineLimits.indices.size());
            LOG_INFO("");

            return std::nullopt;
        }

        RefinedCandidate result;
        result.scanline = std::move(*estimationResultOpt);

        return result;
    }

    VerticalIntrinsicsEstimation VerticalIntrinsicsEstimator::extractResult(
        const int64_t iteration, const PointArray &points, const EndReason endReason
    ) const {
        int64_t unassignedPoints = scanlinePool->getUnassignedPoints();

        if (scanlinePool->anyUnassigned()) {
            LOG_WARN("Warning: Found ", unassignedPoints, " spurious points");
        }

        VerticalScanlinesAssignations scanlinesAssignations = scanlinePool->extractFullSortedScanlineAssignations();

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
        const uint32_t currentScanlineId, const Candidate &candidate,
        const std::optional<RefinedCandidate> &refinedCandidate, const ScanlineAngleBounds &angleBounds
    ) {
        return VerticalScanline{
            .id = currentScanlineId,
            .pointsCount = static_cast<uint64_t>(refinedCandidate->scanline.limits.indices.size()),
            .angle = refinedCandidate->scanline.angle,
            .offset = refinedCandidate->scanline.offset,
            .theoreticalAngleBounds = angleBounds,
            .uncertainty = refinedCandidate->scanline.uncertainty,
            .houghVotes = candidate.hough->cell.votes,
            .houghHash = candidate.hough->cell.hash
        };
    }
} // namespace accurate_ri
