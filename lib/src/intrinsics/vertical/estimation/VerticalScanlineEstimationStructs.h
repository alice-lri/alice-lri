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
            return {
                .bottom = {
                    .lower = angle.ci.lower + asin(offset.ci.lower / maxRange),
                    .upper = angle.ci.lower + asin(offset.ci.lower / minRange)
                },
                .top = {
                    .lower = angle.ci.upper + asin(offset.ci.upper / maxRange),
                    .upper = angle.ci.upper + asin(offset.ci.upper / minRange)
                }
            };
        }
    };
}
