#pragma once
#include <nlohmann/json.hpp>
#include "intrinsics/vertical/VerticalStructs.h"

namespace accurate_ri {
    nlohmann::json offsetAngleToJson(const OffsetAngle &oa);
    OffsetAngle offsetAngleFromJson(const nlohmann::json &j);

    nlohmann::json realMarginToJson(const RealMargin &rm);
    RealMargin realMarginFromJson(const nlohmann::json &j);

    nlohmann::json offsetAngleMarginToJson(const OffsetAngleMargin &oam);
    OffsetAngleMargin offsetAngleMarginFromJson(const nlohmann::json &j);

    nlohmann::json heuristicScanlineToJson(const HeuristicScanline &hsl);
    HeuristicScanline heuristicScanlineFromJson(const nlohmann::json &j);

    nlohmann::json scanlineAngleBoundsToJson(const ScanlineAngleBounds &sab);
    ScanlineAngleBounds scanlineAngleBoundsFromJson(const nlohmann::json &j);

    nlohmann::json scanlineInfoToJson(const ScanlineInfo &si);
    ScanlineInfo scanlineInfoFromJson(const nlohmann::json &j);

    nlohmann::json hashToConflictValueToJson(const HashToConflictValue &htc);
    HashToConflictValue hashToConflictValueFromJson(const nlohmann::json &j);

    nlohmann::json endReasonToJson(const EndReason er);
    EndReason endReasonFromJson(const nlohmann::json &j);

    nlohmann::json verticalIntrinsicsResultToJson(const VerticalIntrinsicsResult &vir);
    VerticalIntrinsicsResult verticalIntrinsicsResultFromJson(const nlohmann::json &j);

    nlohmann::json houghCellToJson(const HoughCell &hc);
    HoughCell houghCellFromJson(const nlohmann::json &j);

    nlohmann::json verticalBoundsToJson(const VerticalBounds &vb);
    VerticalBounds verticalBoundsFromJson(const nlohmann::json &j);

    nlohmann::json horizontalScanlineInfoToJson(const ScanlineHorizontalInfo &shi);
    ScanlineHorizontalInfo horizontalScanlineInfoFromJson(const nlohmann::json &j);

    nlohmann::json horizontalIntrinsicsToJson(const HorizontalIntrinsicsResult &hir);
    HorizontalIntrinsicsResult horizontalIntrinsicsFromJson(const nlohmann::json &j);

    nlohmann::json intrinsicsResultToJson(const IntrinsicsResult &i);
    IntrinsicsResult intrinsicsResultFromJson(const nlohmann::json &j);
}
