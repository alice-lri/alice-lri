#pragma once
#include <iomanip>
#include <unordered_set>
#include <optional>
#include "math/Stats.h"
#include "accurate_ri/public_structs.hpp"

// TODO move structs to their own places (locality of behaviour)
namespace accurate_ri {

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

    struct ScanlineFitResult {
        std::optional<Stats::WLSResult> fit;
        std::optional<ScanlineLimits> limits;
        bool success = false;
        bool validCi = false;
    };

    // TODO objetive: remove this and construct ScanlineInfo through the process
    struct ScanlineEstimationResult {
        bool heuristic;
        double uncertainty;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineLimits limits;

        [[nodiscard]] ScanlineAngleBounds toAngleBounds(
            const double minRange, const double maxRange
        ) const {
            return {
                .bottom = {
                    .lower = ci.angle.lower + asin(ci.offset.lower / maxRange),
                    .upper = ci.angle.lower + asin(ci.offset.lower / minRange)
                },
                .top = {
                    .lower = ci.angle.upper + asin(ci.offset.upper / maxRange),
                    .upper = ci.angle.upper + asin(ci.offset.upper / minRange)
                }
            };
        }
    };

    struct ValueConfInterval {
        double value;
        RealMargin ci;
    };

    // TODO move elsewhere
    inline std::ostream &operator<<(std::ostream &os, const RealMargin &margin) {
        os << std::fixed << std::setprecision(5) << "[" << margin.lower << ", " << margin.upper << "]";
        return os;
    }
}
