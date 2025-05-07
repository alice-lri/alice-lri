#include "JsonConverters.h"

namespace accurate_ri {
    nlohmann::json offsetAngleToJson(const OffsetAngle &oa) {
        nlohmann::json j;
        j["offset"] = oa.offset;
        j["angle"] = oa.angle;
        return j;
    }

    nlohmann::json realMarginToJson(const RealMargin &rm) {
        nlohmann::json j;
        j["lower"] = rm.lower;
        j["upper"] = rm.upper;
        return j;
    }

    nlohmann::json offsetAngleMarginToJson(const OffsetAngleMargin &oam) {
        nlohmann::json j;
        j["offset"] = realMarginToJson(oam.offset);
        j["angle"] = realMarginToJson(oam.angle);
        return j;
    }

    nlohmann::json scanlineLimitsToJson(const ScanlineLimits &sl) {
        nlohmann::json j;
        j["indices"] = std::vector<int>(sl.indices.data(), sl.indices.data() + sl.indices.size());
        j["mask"] = std::vector<bool>(sl.mask.data(), sl.mask.data() + sl.mask.size());
        j["lowerLimit"] = std::vector<double>(sl.lowerLimit.data(), sl.lowerLimit.data() + sl.lowerLimit.size());
        j["upperLimit"] = std::vector<double>(sl.upperLimit.data(), sl.upperLimit.data() + sl.upperLimit.size());
        return j;
    }

    nlohmann::json heuristicScanlineToJson(const HeuristicScanline &hsl) {
        nlohmann::json j;
        j["offset"] = hsl.offset;
        j["offsetCi"] = realMarginToJson(hsl.offsetCi);
        j["dependencies"] = hsl.dependencies;
        return j;
    }

    nlohmann::json scanlineAngleBoundsToJson(const ScanlineAngleBounds &sab) {
        nlohmann::json j;
        j["bottom"] = realMarginToJson(sab.bottom);
        j["top"] = realMarginToJson(sab.top);
        return j;
    }

    nlohmann::json scanlineInfoToJson(const ScanlineInfo &si) {
        nlohmann::json j;
        j["offset"] = si.values.offset;
        j["angle"] = si.values.angle;
        j["hough_votes"] = si.houghVotes;
        j["hash"] = si.houghHash;
        j["count"] = si.pointsCount;
        j["lower_min_theoretical_angle"] = si.theoreticalAngleBounds.bottom.lower;
        j["lower_max_theoretical_angle"] = si.theoreticalAngleBounds.bottom.upper;
        j["upper_min_theoretical_angle"] = si.theoreticalAngleBounds.top.lower;
        j["upper_max_theoretical_angle"] = si.theoreticalAngleBounds.top.upper;
        j["uncertainty"] = si.uncertainty;
        j["angle_ci"] = nlohmann::json::array();
        j["angle_ci"].push_back(si.ci.angle.lower);
        j["angle_ci"].push_back(si.ci.angle.upper);
        j["offset_ci"] = nlohmann::json::array();
        j["offset_ci"].push_back(si.ci.offset.lower);
        j["offset_ci"].push_back(si.ci.offset.upper);
        j["dependencies"] = si.dependencies;
        j["last_scanline_assignment"] = false;
        return j;
    }

    nlohmann::json hashToConflictValueToJson(const HashToConflictValue &htc) {
        nlohmann::json j;
        // Convert unordered_set to vector for JSON serialization
        j["conflictingScanlines"] = std::vector<uint32_t>(htc.conflictingScanlines.begin(),
                                                          htc.conflictingScanlines.end());
        j["votes"] = htc.votes;
        return j;
    }

    nlohmann::json endReasonToJson(const EndReason er) {
        std::string endReasonStr;
        switch (er) {
            case EndReason::ALL_ASSIGNED: endReasonStr = "ALL_ASSIGNED";
                break;
            case EndReason::MAX_ITERATIONS: endReasonStr = "MAX_ITERATIONS";
                break;
            case EndReason::NO_MORE_PEAKS: endReasonStr = "NO_MORE_PEAKS";
                break;
            default: endReasonStr = "UNKNOWN";
                break; // Handle unexpected values if needed
        }
        return endReasonStr;
    }

    nlohmann::json verticalIntrinsicsResultToJson(const VerticalIntrinsicsResult &vir) {
        nlohmann::json j;
        j["iterations"] = vir.iterations;
        j["scanlines_count"] = vir.scanlinesCount;
        j["end_reason"] = endReasonToJson(vir.endReason);
        j["unassigned_points"] = vir.unassignedPoints;
        j["points_count"] = vir.pointsCount;
        j["scanlines_attributes"] = nlohmann::json::array();
        for (const auto &scanline: vir.fullScanlines.scanlines) {
            j["scanlines_attributes"].push_back(scanlineInfoToJson(scanline));
        }
        return j;
    }

    nlohmann::json houghCellToJson(const HoughCell &hc) {
        nlohmann::json j;
        j["maxOffsetIndex"] = hc.maxOffsetIndex;
        j["maxAngleIndex"] = hc.maxAngleIndex;
        j["maxValues"] = offsetAngleToJson(hc.maxValues);
        j["votes"] = hc.votes;
        j["hash"] = hc.hash;
        return j;
    }

    nlohmann::json verticalBoundsToJson(const VerticalBounds &vb) {
        nlohmann::json j;
        j["phis"] = std::vector<double>(vb.phis.data(), vb.phis.data() + vb.phis.size());
        j["correction"] = std::vector<double>(vb.correction.data(), vb.correction.data() + vb.correction.size());
        j["final"] = std::vector<double>(vb.final.data(), vb.final.data() + vb.final.size());
        return j;
    }

    nlohmann::json horizontalScanlineInfoToJson(const ScanlineHorizontalInfo &shi) {
        nlohmann::json j;
        j["resolution"] = shi.resolution;
        j["offset"] = shi.offset;
        j["heuristic"] = shi.heuristic;
        return j;
    }

    nlohmann::json horizontalIntrinsicsToJson(const HorizontalIntrinsicsResult &hir) {
        nlohmann::json j;
        j["scanlines_attributes"] = nlohmann::json::array();
        for (const auto &scanline: hir.scanlines) {
            j["scanlines_attributes"].push_back(horizontalScanlineInfoToJson(scanline));
        }
        return j;
    }

    nlohmann::json intrinsicsResultToJson(const IntrinsicsResult &i) {
        nlohmann::json j;
        j["vertical"] = verticalIntrinsicsResultToJson(i.vertical);
        j["horizontal"] = horizontalIntrinsicsToJson(i.horizontal);
        return j;
    }
} // accurate_ri