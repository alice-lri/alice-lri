#pragma once
#include <cstdint>
#include <vector>

namespace accurate_ri {
    struct RealMargin {
        double lower;
        double upper;

        [[nodiscard]] double diff() const {
            return upper - lower;
        }

        inline void clampBoth(const double minValue, const double maxValue) {
            lower = std::clamp(lower, minValue, maxValue);
            upper = std::clamp(upper, minValue, maxValue);
        }
    };

    struct ScanlineAngleBounds {
        RealMargin bottom;
        RealMargin top;
    };

    struct OffsetAngleMargin {
        RealMargin offset;
        RealMargin angle;
    };

    struct OffsetAngle {
        double offset;
        double angle;
    };

    struct ScanlineInfo {
        uint32_t id;
        uint64_t pointsCount;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineAngleBounds theoreticalAngleBounds;
        std::vector<uint32_t> dependencies;
        double uncertainty;
        double houghVotes;
        uint64_t houghHash;
    };

    struct FullScanlines {
        std::vector<ScanlineInfo> scanlines;
        std::vector<int> pointsScanlinesIds;
    };

    enum class EndReason {
        ALL_ASSIGNED,
        MAX_ITERATIONS,
        NO_MORE_PEAKS
    };

    struct ScanlineHorizontalInfo { // TODO maybe rename to HorizontalScanlineInfo
        int32_t resolution;
        double offset;
        bool heuristic;
    };

    struct HorizontalIntrinsicsResult {
        std::vector<ScanlineHorizontalInfo> scanlines;
    };

    struct VerticalIntrinsicsResult {
        uint32_t iterations = 0;
        uint32_t scanlinesCount = 0;
        uint32_t unassignedPoints = 0;
        uint32_t pointsCount = 0;
        EndReason endReason = EndReason::MAX_ITERATIONS;
        FullScanlines fullScanlines;
    };

    struct IntrinsicsResult {
        VerticalIntrinsicsResult vertical;
        HorizontalIntrinsicsResult horizontal;
    };

    struct RangeImage {
        int32_t width;
        int32_t height;
        std::vector<double> data;
    };
}
