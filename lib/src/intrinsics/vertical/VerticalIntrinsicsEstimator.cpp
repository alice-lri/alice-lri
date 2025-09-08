#include "VerticalIntrinsicsEstimator.h"

#include <accurate_ri/accurate_ri.hpp>
#include <algorithm>
#include <queue>
#include <ranges>
#include "helper/VerticalLogging.h"
#include "utils/Utils.h"
#include "VerticalStructs.h"
#include "utils/TestUtils.h"
#include "utils/Timer.h"
#include <nlohmann/json.hpp>
#include "Constants.h"
#include "BuildOptions.h"
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

    // TODO perhaps further split this function
    VerticalIntrinsicsResult VerticalIntrinsicsEstimator::estimate(const PointArray &points) {
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

            scanlinePool->removeVotes(points, refinedCandidate->scanline.limits.indices);

            const ScanlineInfo scanlineInfo = ScanlineInfo{
                .id = currentScanlineId,
                .pointsCount = static_cast<uint64_t>(refinedCandidate->scanline.limits.indices.size()),
                .values = refinedCandidate->scanline.values,
                .ci = refinedCandidate->scanline.ci,
                .theoreticalAngleBounds = angleBounds,
                .uncertainty = refinedCandidate->scanline.uncertainty,
                .houghVotes = candidate.hough->cell.votes,
                .houghHash = candidate.hough->cell.hash
            };

            scanlinePool->assignScanline(scanlineInfo, refinedCandidate->scanline.limits.indices);

            VerticalLogging::logScanlineAssignation(scanlineInfo);
            LOG_INFO("Number of unassigned points: ", scanlinePool->getUnassignedPoints());
            LOG_INFO("");

            currentScanlineId++;
        }

        const VerticalIntrinsicsResult result = extractResult(iteration, points, endReason);
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
        const OffsetAngleMargin &margin = candidate.hough->margin;
        const HoughCell &houghMax = candidate.hough->cell;

        VerticalBounds errorBounds = VerticalScanlineLimits::computeErrorBounds(points, houghMax.maxValues.offset);
        ScanlineLimits scanlineLimits = VerticalScanlineLimits::computeScanlineLimits(
            points, errorBounds.final, houghMax.maxValues, margin
        );

        LOG_INFO("Minimum limit width (Hough): ", (scanlineLimits.upperLimit - scanlineLimits.lowerLimit).minCoeff());

        // TODO remove
        const std::vector<ScanlineInfo> debugScanlines = scanlinePool->getUnsortedScanlinesCopy();

        VerticalLogging::plotDebugInfo(
            points, debugScanlines, scanlineLimits, scanlinePool->getPointsScanlinesIds(), iteration, "hough_",
            houghMax.maxValues, 0
        );

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
        result.scanline = Utils::force_move(*estimationResultOpt);

        VerticalLogging::plotDebugInfo(
            points, debugScanlines, result.scanline.limits, scanlinePool->getPointsScanlinesIds(), iteration, "fit_",
            result.scanline.values, result.scanline.uncertainty
        );

        return result;
    }

    VerticalIntrinsicsResult VerticalIntrinsicsEstimator::extractResult(
        const int64_t iteration, const PointArray &points, const EndReason endReason
    ) const {
        int64_t unassignedPoints = scanlinePool->getUnassignedPoints();

        if (scanlinePool->anyUnassigned()) {
            LOG_WARN("Warning: Found ", unassignedPoints, " spurious points");
        }

        FullScanlines fullScanlines = scanlinePool->extractFullSortedScanlines();

        LOG_INFO("Number of scanlines: ", fullScanlines.scanlines.size());
        LOG_INFO("Number of unassigned points: ", unassignedPoints);

        return VerticalIntrinsicsResult{
            .iterations = static_cast<int32_t>(iteration),
            .scanlinesCount = static_cast<int32_t>(fullScanlines.scanlines.size()),
            .unassignedPoints = static_cast<int32_t>(unassignedPoints),
            .pointsCount = static_cast<int32_t>(points.size()),
            .endReason = endReason,
            .fullScanlines = std::move(fullScanlines)
        };
    }
} // namespace accurate_ri
