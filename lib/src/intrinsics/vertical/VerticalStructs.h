#pragma once
#include <cstdint>
#include <eigen3/Eigen/Dense>

namespace accurate_ri {
    struct OffsetAngle {
        double offset;
        double angle;
    };

    struct HoughCell {
        uint64_t maxOffsetIndex;
        uint64_t maxAngleIndex;
        OffsetAngle maxValues;
        double votes;
        uint64_t hash;
    };

    struct VerticalBounds {
        Eigen::ArrayXd phis;
        Eigen::ArrayXd correction;
        Eigen::ArrayXd final;
    };

    struct ScanlineLimits {
        Eigen::ArrayXi indices;
        Eigen::ArrayX<bool> mask;
        Eigen::ArrayXd lowerLimit;
        Eigen::ArrayXd upperLimit;
    };

    struct RealMargin {
        double lower;
        double upper;
    };

    struct OffsetAngleMargin {
        RealMargin offset;
        RealMargin angle;
    };

    struct LinearFitResult {
        OffsetAngle values;
        OffsetAngle variance;
        OffsetAngleMargin ci;
        double aic;
    };

    struct ScanlineFitResult {
        bool success = false;
        bool ciTooWide = false;
        std::optional<LinearFitResult> fit;
        std::optional<ScanlineLimits> limits;
    };
}
