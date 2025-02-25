#pragma once
#include <cstdint>
#include <iomanip>
#include <unordered_set>
#include <eigen3/Eigen/Dense>
#include <optional>

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

    // TODO this containing indices and mask is probably not intuitive
    struct ScanlineLimits {
        Eigen::ArrayXi indices;
        Eigen::ArrayX<bool> mask;
        Eigen::ArrayXd lowerLimit;
        Eigen::ArrayXd upperLimit;
    };

    struct RealMargin {
        double lower;
        double upper;

        [[nodiscard]] double diff() const {
            return upper - lower;
        }
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

    // TODO objetive: remove this and construct ScanlineInfo through the process
    struct ScanlineEstimationResult {
        bool heuristic;
        double uncertainty;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineLimits limits;
        std::vector<uint32_t> dependencies;
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
        std::vector<uint32_t> dependencies;
    };

    struct ScanlineAngleBounds {
        RealMargin bottom;
        RealMargin top;
    };

    struct ScanlineInfo {
        uint32_t scanlineId;
        uint64_t pointsCount;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineAngleBounds theoreticalAngleBounds;
        std::vector<uint32_t> dependencies;
        double uncertainty;
        double houghVotes;
        uint64_t houghHash;
    };

    struct HashToConflictValue {
        std::unordered_set<uint32_t> conflictingScanlines;
        double votes;
    };

    enum class EndReason {
        ALL_ASSIGNED,
        MAX_ITERATIONS,
        NO_MORE_PEAKS
    };

    struct VerticalIntrinsicsResult {
        uint32_t iterations;
        uint32_t scanlinesCount;
        uint32_t unassignedPoints;
        uint32_t pointsCount;
        EndReason endReason;
        std::vector<ScanlineInfo> scanlines;
        Eigen::ArrayXi pointsScanlinesIds;
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
