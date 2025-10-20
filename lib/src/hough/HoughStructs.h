#pragma once
#include <cstdint>

enum class HoughOperation {
    ADD, SUBTRACT
};

enum class HoughMode {
    VOTES_ONLY, VOTES_AND_HASHES
};

struct HoughCell {
    uint64_t maxOffsetIndex;
    uint64_t maxAngleIndex;
    double maxOffset;
    double maxAngle;
    int64_t votes;
    uint64_t hash;
};

struct VerticalMargin {
    double offset;
    double angle;
};

struct HoughScanlineEstimation {
    HoughCell cell;
    VerticalMargin margin;
};