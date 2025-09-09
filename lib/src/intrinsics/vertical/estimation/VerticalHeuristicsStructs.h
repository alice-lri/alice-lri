#pragma once
#include <cstdint>
#include <optional>
#include "intrinsics/vertical/VerticalStructs.h"

namespace accurate_ri {
    struct HeuristicScanline {
        ValueConfInterval offset;
        ValueConfInterval angle;
    };

    struct HeuristicSupportScanline {
        uint32_t id;
        double distance;
    };

    struct HeuristicSupportScanlinePair {
        std::optional<HeuristicSupportScanline> top;
        std::optional<HeuristicSupportScanline> bottom;
    };
}
