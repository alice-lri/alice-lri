#pragma once
#include <cstdint>
#include <vector>

namespace accurate_ri {
    struct ScanlineHorizontalInfo {
        int32_t resolution;
        double offset;
        bool heuristic;
    };

    struct HorizontalIntrinsicsResult {
        std::vector<ScanlineHorizontalInfo> scanlines;
    };
}
