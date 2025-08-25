#pragma once
#include <cstdint>
#include <iomanip>
#include <unordered_set>
#include <optional>
#include "math/Stats.h"
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    struct HoughCell {
        uint64_t maxOffsetIndex;
        uint64_t maxAngleIndex;
        OffsetAngle maxValues;
        int64_t votes;
        uint64_t hash;
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

    struct HoughScanlineEstimation {
        HoughCell cell;
        OffsetAngleMargin margin;
    };

    struct ScanlineFitResult {
        std::optional<Stats::WLSResult> fit;
        std::optional<ScanlineLimits> limits;
        bool success = false;
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

    struct ScanlineIntersectionInfo {
        const Eigen::ArrayX<bool> empiricalIntersectionMask;
        const Eigen::ArrayX<bool> theoreticalIntersectionMask;
        const bool empiricalIntersection;
        const bool theoreticalIntersection;

        inline bool anyIntersection() const {
            return empiricalIntersection || theoreticalIntersection;
        }

        inline bool anyIntersection(const uint32_t i) const {
            return empiricalIntersectionMask[i] || theoreticalIntersectionMask[i];
        }
    };

    struct HeuristicScanline {
        double offset;
        RealMargin offsetCi;
    };

    struct HashToConflictValue {
        std::unordered_set<uint32_t> conflictingScanlines;
        int64_t votes;
    };

    struct ScanlineConflictsResult {
        bool shouldReject;
        Eigen::ArrayXi conflictingScanlines;
    };

    // TODO move elsewhere
    template<typename T>
    std::ostream &operator<<(std::ostream &os, const std::unordered_set<T> &set) {
        os << std::fixed << std::setprecision(5) << "{";

        bool firstElement = true;
        for (const auto &element: set) {
            if (!firstElement) {
                os << ", ";
            }
            os << element;
            firstElement = false;
        }

        os << "}";
        return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const RealMargin &margin) {
        os << std::fixed << std::setprecision(5) << "[" << margin.lower << ", " << margin.upper << "]";
        return os;
    }

    template<typename T>
    std::ostream &operator<<(std::ostream &os, const Eigen::ArrayX<T> &array) {
        os << std::fixed << std::setprecision(5) << "[";

        bool firstElement = true;
        for (const auto &element: array) {
            if (!firstElement) {
                os << ", ";
            }
            os << element;
            firstElement = false;
        }

        os << "]";
        return os;
    }
}
