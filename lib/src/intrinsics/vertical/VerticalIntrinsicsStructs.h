#pragma once
#include <optional>
#include <vector>
#include <Eigen/Core>
#include "accurate_ri/public_structs.hpp"
#include "hough/HoughStructs.h"

namespace accurate_ri {
    struct VerticalScanline {
        uint32_t id;
        uint64_t pointsCount;
        ValueConfInterval angle;
        ValueConfInterval offset;
        ScanlineAngleBounds theoreticalAngleBounds;
        double uncertainty;
        bool heuristic;
        HoughScanlineEstimation hough;
    };

    struct VerticalScanlinesAssignations {
        std::vector<VerticalScanline> scanlines;
        std::vector<int> pointsScanlinesIds;
    };

    struct VerticalIntrinsicsEstimation {
        int32_t iterations = 0;
        int32_t unassignedPoints = 0;
        int32_t pointsCount = 0;
        EndReason endReason = EndReason::ALL_ASSIGNED;
        VerticalScanlinesAssignations scanlinesAssignations;
    };

    struct VerticalBounds {
        Eigen::ArrayXd phis;
        Eigen::ArrayXd correction;
        Eigen::ArrayXd final;
    };

    struct ScanlineLimits {
        Eigen::ArrayXi indices;
        Eigen::ArrayX<bool> mask;
    };

    struct VerticalScanlineHoughCandidate {
        std::optional<HoughScanlineEstimation> estimation = std::nullopt;
        std::optional<EndReason> endReason = std::nullopt;
        bool available = false;
    };

    struct VerticalScanlineCandidate {
        VerticalScanline scanline;
        ScanlineLimits limits;
    };
}
