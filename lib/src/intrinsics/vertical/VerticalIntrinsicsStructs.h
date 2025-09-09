#pragma once
#include <vector>
#include <Eigen/Core>
#include "utils/CommonStructs.h"

namespace accurate_ri {

    struct VerticalMargin {
        double offset;
        double angle;
    };

    struct VerticalScanline {
        uint32_t id;
        uint64_t pointsCount;
        ValueConfInterval angle;
        ValueConfInterval offset;
        ScanlineAngleBounds theoreticalAngleBounds;
        double uncertainty;
        int64_t houghVotes;
        uint64_t houghHash;
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

    // TODO this containing indices and mask is probably not intuitive
    struct ScanlineLimits {
        Eigen::ArrayXi indices;
        Eigen::ArrayX<bool> mask;
        Eigen::ArrayXd lowerLimit;
        Eigen::ArrayXd upperLimit; // TODO just realized the limits are not really neccessary to store
    };
}
