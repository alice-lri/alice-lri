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

    nlohmann::json linearFitResultToJson(const LinearFitResult &lfr) {
        nlohmann::json j;
        j["values"] = offsetAngleToJson(lfr.values);
        j["variance"] = offsetAngleToJson(lfr.variance);
        j["ci"] = offsetAngleMarginToJson(lfr.ci);
        j["aic"] = lfr.aic;
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

    nlohmann::json scanlineFitResultToJson(const ScanlineFitResult &sfr) {
        nlohmann::json j;
        j["success"] = sfr.success;
        j["ciTooWide"] = sfr.ciTooWide;
        if (sfr.fit.has_value()) {
            j["fit"] = linearFitResultToJson(sfr.fit.value());
        } else {
            j["fit"] = nullptr; // Or nlohmann::json::null() if you prefer explicit null
        }
        if (sfr.limits.has_value()) {
            j["limits"] = scanlineLimitsToJson(sfr.limits.value());
        } else {
            j["limits"] = nullptr; // Or nlohmann::json::null()
        }
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
        j["scanlineId"] = si.scanlineId;
        j["pointsCount"] = si.pointsCount;
        j["values"] = offsetAngleToJson(si.values);
        j["ci"] = offsetAngleMarginToJson(si.ci);
        j["theoreticalAngleBounds"] = scanlineAngleBoundsToJson(si.theoreticalAngleBounds);
        j["dependencies"] = si.dependencies;
        j["uncertainty"] = si.uncertainty;
        j["houghVotes"] = si.houghVotes;
        j["houghHash"] = si.houghHash;
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
        j["scanlinesCount"] = vir.scanlinesCount;
        j["unassignedPoints"] = vir.unassignedPoints;
        j["pointsCount"] = vir.pointsCount;
        j["endReason"] = endReasonToJson(vir.endReason);
        j["scanlines"] = nlohmann::json::array();
        for (const auto &scanline: vir.scanlines) {
            j["scanlines"].push_back(scanlineInfoToJson(scanline));
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
} // accurate_ri