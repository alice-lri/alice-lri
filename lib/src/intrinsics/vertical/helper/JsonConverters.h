#pragma once
#include <nlohmann/json.hpp>
#include "intrinsics/vertical/VerticalStructs.h"

namespace accurate_ri {
    nlohmann::json offsetAngleToJson(const OffsetAngle &oa);

    nlohmann::json realMarginToJson(const RealMargin &rm);

    nlohmann::json offsetAngleMarginToJson(const OffsetAngleMargin &oam);

    nlohmann::json linearFitResultToJson(const LinearFitResult &lfr);

    nlohmann::json scanlineLimitsToJson(const ScanlineLimits &sl);

    nlohmann::json scanlineFitResultToJson(const ScanlineFitResult &sfr);

    nlohmann::json heuristicScanlineToJson(const HeuristicScanline &hsl);

    nlohmann::json scanlineAngleBoundsToJson(const ScanlineAngleBounds &sab);

    nlohmann::json scanlineInfoToJson(const ScanlineInfo &si);

    nlohmann::json hashToConflictValueToJson(const HashToConflictValue &htc);

    nlohmann::json endReasonToJson(const EndReason er);

    nlohmann::json verticalIntrinsicsResultToJson(const VerticalIntrinsicsResult &vir);

    nlohmann::json houghCellToJson(const HoughCell &hc);

    nlohmann::json verticalBoundsToJson(const VerticalBounds &vb);
}
