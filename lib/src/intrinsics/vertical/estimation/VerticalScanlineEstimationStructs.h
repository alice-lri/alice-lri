#pragma once
#include <cstdint>
#include <optional>

#include "intrinsics/vertical/VerticalIntrinsicsStructs.h"
#include "math/Stats.h"
#include "utils/CommonStructs.h"

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

    struct ScanlineFitResult {
        std::optional<Stats::WLSResult> fit;
        std::optional<ScanlineLimits> limits;
        bool success = false;
        bool validCi = false;
    };

    struct VerticalScanlineEstimation {
        bool heuristic;
        double uncertainty;
        ValueConfInterval offset;
        ValueConfInterval angle;
        ScanlineLimits limits;

        [[nodiscard]] ScanlineAngleBounds toAngleBounds(
            const double minRange, const double maxRange
        ) const {
            const double lowerLineA = angle.ci.lower + asin(offset.ci.lower / maxRange);
            const double lowerLineB = angle.ci.lower + asin(offset.ci.lower / minRange);
            const double upperLineA = angle.ci.upper + asin(offset.ci.upper / maxRange);
            const double upperLineB = angle.ci.upper + asin(offset.ci.upper / minRange);

            return {
                .lowerLine = {
                    .lower = std::min(lowerLineA, lowerLineB),
                    .upper = std::max(lowerLineA, lowerLineB)
                },
                .upperLine = {
                    .lower = std::min(upperLineA, upperLineB),
                    .upper = std::max(upperLineA, upperLineB)
                }
            };
        }
    };
}
