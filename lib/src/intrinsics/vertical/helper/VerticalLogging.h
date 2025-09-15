#pragma once
#include <string>
#include "utils/logger/Logger.h"
#include <filesystem>
#include <variant>
#include <Eigen/Dense>

#include "hough/HoughTransform.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace alice_lri::VerticalLogging {

    inline void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough) {
        LOG_INFO("==| Parameters |==");
        LOG_INFO("Number of points: ", points.size());
        LOG_INFO("Coord distortion compensation strategy: ", "upper_bound");
        LOG_INFO("Hough model: ", "arcsin");
        LOG_INFO("Voting strategy: ", "single");
        LOG_INFO("Noise radius use square: ", "False");
        LOG_INFO("Fit type: ", "linear");
        LOG_INFO(
            "Offset min: ", hough.getXMin(), ", Offset max: ", hough.getXMax(), ", Offset res: ", hough.getXStep()
        );
        LOG_INFO("Angle min: ", hough.getYMin(), ", Angle max: ", hough.getYMax(), ", Angle res: ", hough.getYStep());
        LOG_INFO("Coords eps: ", points.getCoordsEps());
        LOG_INFO("");
        LOG_INFO("==| Execution |==");
    }

    inline void logHoughInfo(const int64_t iteration, const HoughCell &houghMax) {
        LOG_INFO("ITERATION ", iteration);
        LOG_INFO(
            "Offset: ", houghMax.maxOffset, ", Angle: ", houghMax.maxAngle, ", Votes: ", houghMax.votes,
            ", Hash: ", houghMax.hash, ", Hough indices: [", houghMax.maxAngleIndex, "   ", houghMax.maxOffsetIndex,
            "]"
        );
    }

    inline void logScanlineAssignation(const VerticalScanline& scanline) {
        LOG_INFO("Scanline ", scanline.id, " assigned with ", scanline.pointsCount, " points");
        LOG_INFO(
            "Scanline parameters: Offset: ", scanline.offset.value, ", Angle: ", scanline.angle.value,
            ", Votes: ", scanline.hough.cell.votes, ", Count: ", scanline.pointsCount,
            ", Lower min theoretical angle: ", scanline.theoreticalAngleBounds.lowerLine.lower,
            ", Lower max theoretical angle: ", scanline.theoreticalAngleBounds.lowerLine.upper,
            ", Upper min theoretical angle: ", scanline.theoreticalAngleBounds.upperLine.lower,
            ", Upper max theoretical angle: ", scanline.theoreticalAngleBounds.upperLine.upper,
            ", Uncertainty: ", scanline.uncertainty
        );
    }

    inline void logIntersectionProblem(
        const ScanlineIntersectionInfo& intersection, const VerticalScanlineCandidate& candidate
    ) {
        LOG_INFO("Possible problem detected");
        LOG_INFO(
            "Intersects other scanline: ", intersection.flags.empiricalIntersection? "True": "False",
            ", Intersects theoretically: ", intersection.flags.theoreticalIntersection,
            ", Fit success: ", "True",
            ", Points in scanline: ", candidate.limits.indices.size(), " vs ", candidate.scanline.hough.cell.votes
        );

        LOG_INFO(
            "Intersects with scanlines: ", intersection.conflictingIds,
            ", Current scanline uncertainty: ", candidate.scanline.uncertainty,
            ", Conflicting scanlines uncertainties: ", intersection.conflictingUncertainties
        );
    }
}
