#pragma once
#include <vector>

struct HorizontalScanline {
    int32_t resolution;
    double offset;
    double thetaOffset;
    bool heuristic;
};

struct HorizontalIntrinsicsEstimation {
    std::vector<HorizontalScanline> scanlines;
};