#pragma once
#include <cstdint>

namespace accurate_ri::Constant {
    constexpr uint64_t VERTICAL_MAX_ITERATIONS = 10000;
    constexpr double MAX_OFFSET = 0.5;
    constexpr double OFFSET_STEP = 1e-3;
    constexpr double ANGLE_STEP = 1e-4;
    constexpr uint64_t VERTICAL_MAX_FIT_ATTEMPTS = 10;

    constexpr int32_t MAX_RESOLUTION = 10000;
    constexpr double INV_RANGES_BREAK_THRESHOLD = 1e-2;
    constexpr int32_t HORIZONTAL_MIN_POINTS_PER_SCANLINE = 16;
}
