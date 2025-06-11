#pragma once
#include <cstdint>

namespace accurate_ri::Constant {
    constexpr int32_t MAX_RESOLUTION = 10000;

    // TODO infer this more smartly, use gaussian thingies or something to derive bounds w.r.t to points
    constexpr int32_t MAX_RESOLUTION_DELTA = 50;

    // TODO derive this more elegantly, assuming a max offset or something
    constexpr double INV_RANGES_BREAK_THRESHOLD = 1e-2;

    constexpr int32_t HORIZONTAL_MIN_POINTS_PER_SCANLINE = 16;

    constexpr double MAX_OFFSET = 0.5;
}
