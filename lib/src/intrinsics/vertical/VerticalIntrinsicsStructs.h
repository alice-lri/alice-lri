#pragma once
#include <cstdint>
#include <vector>

struct RealMargin { // TODO refactor to value and ci or something and use also in heuristics
    double lower;
    double upper;

    [[nodiscard]] double diff() const {
        return upper - lower;
    }

    void clampBoth(const double minValue, const double maxValue) {
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

struct VerticalScanline {
    uint32_t id;
    uint64_t pointsCount;
    OffsetAngle values;
    OffsetAngleMargin ci;
    ScanlineAngleBounds theoreticalAngleBounds;
    double uncertainty;
    int64_t houghVotes;
    uint64_t houghHash;
};

enum class EndReason {
    ALL_ASSIGNED, MAX_ITERATIONS, NO_MORE_PEAKS
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