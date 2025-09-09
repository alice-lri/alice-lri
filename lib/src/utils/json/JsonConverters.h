#pragma once
#include <nlohmann/json.hpp>

#include "accurate_ri/public_structs.hpp"
#include "hough/HoughTransform.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "intrinsics/vertical/conflict/ScanlineConflictStructs.h"

namespace accurate_ri {
    nlohmann::json offsetAngleToJson(const OffsetAngle &oa);
    OffsetAngle offsetAngleFromJson(const nlohmann::json &j);

    nlohmann::json realMarginToJson(const RealMargin &rm);
    RealMargin realMarginFromJson(const nlohmann::json &j);

    nlohmann::json offsetAngleMarginToJson(const OffsetAngleMargin &oam);
    OffsetAngleMargin offsetAngleMarginFromJson(const nlohmann::json &j);

    nlohmann::json heuristicScanlineToJson(const ValueConfInterval &hsl);
    ValueConfInterval heuristicScanlineFromJson(const nlohmann::json &j);

    nlohmann::json scanlineAngleBoundsToJson(const ScanlineAngleBounds &sab);
    ScanlineAngleBounds scanlineAngleBoundsFromJson(const nlohmann::json &j);

    nlohmann::json scanlineInfoToJson(const VerticalScanline &si);
    VerticalScanline scanlineInfoFromJson(const nlohmann::json &j);

    nlohmann::json hashToConflictValueToJson(const HashToConflictValue &htc);
    HashToConflictValue hashToConflictValueFromJson(const nlohmann::json &j);

    nlohmann::json endReasonToJson(EndReason er);
    EndReason endReasonFromJson(const nlohmann::json &j);

    nlohmann::json verticalIntrinsicsResultToJson(const VerticalIntrinsicsEstimation &vir);
    VerticalIntrinsicsEstimation verticalIntrinsicsResultFromJson(const nlohmann::json &j);

    nlohmann::json houghCellToJson(const HoughCell &hc);
    HoughCell houghCellFromJson(const nlohmann::json &j);

    nlohmann::json verticalBoundsToJson(const VerticalBounds &vb);
    VerticalBounds verticalBoundsFromJson(const nlohmann::json &j);

    nlohmann::json horizontalScanlineInfoToJson(const ScanlineHorizontalInfo &shi);
    ScanlineHorizontalInfo horizontalScanlineInfoFromJson(const nlohmann::json &j);

    nlohmann::json horizontalIntrinsicsToJson(const HorizontalIntrinsicsResult &hir);
    HorizontalIntrinsicsResult horizontalIntrinsicsFromJson(const nlohmann::json &j);

    nlohmann::json intrinsicsResultToJson(const Intrinsics &i);
    Intrinsics intrinsicsResultFromJson(const nlohmann::json &j);
}
