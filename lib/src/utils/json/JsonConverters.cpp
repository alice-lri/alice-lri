#include "JsonConverters.h"

namespace accurate_ri {
    nlohmann::json offsetAngleToJson(const OffsetAngle &oa) {
        nlohmann::json j;
        j["offset"] = oa.offset;
        j["angle"] = oa.angle;
        return j;
    }

    OffsetAngle offsetAngleFromJson(const nlohmann::json &j) {
        return OffsetAngle{j.at("offset"), j.at("angle")};
    }

    nlohmann::json realMarginToJson(const RealMargin &rm) {
        nlohmann::json j;
        j["lower"] = rm.lower;
        j["upper"] = rm.upper;
        return j;
    }

    RealMargin realMarginFromJson(const nlohmann::json &j) {
        return RealMargin{j.at("lower"), j.at("upper")};
    }

    nlohmann::json offsetAngleMarginToJson(const OffsetAngleMargin &oam) {
        nlohmann::json j;
        j["offset"] = realMarginToJson(oam.offset);
        j["angle"] = realMarginToJson(oam.angle);
        return j;
    }

    OffsetAngleMargin offsetAngleMarginFromJson(const nlohmann::json &j) {
        return OffsetAngleMargin{realMarginFromJson(j.at("offset")), realMarginFromJson(j.at("angle"))};
    }


    nlohmann::json heuristicScanlineToJson(const ValueConfInterval &hsl) {
        nlohmann::json j;
        j["offset"] = hsl.value;
        j["offsetCi"] = realMarginToJson(hsl.ci);
        return j;
    }

    ValueConfInterval heuristicScanlineFromJson(const nlohmann::json &j) {
        return ValueConfInterval{
            j.at("offset"),
            realMarginFromJson(j.at("offsetCi"))
        };
    }

    nlohmann::json scanlineAngleBoundsToJson(const ScanlineAngleBounds &sab) {
        nlohmann::json j;
        j["bottom"] = realMarginToJson(sab.bottom);
        j["top"] = realMarginToJson(sab.top);
        return j;
    }

    ScanlineAngleBounds scanlineAngleBoundsFromJson(const nlohmann::json &j) {
        return ScanlineAngleBounds{realMarginFromJson(j.at("bottom")), realMarginFromJson(j.at("top"))};
    }

    nlohmann::json scanlineInfoToJson(const ScanlineInfo &si) {
        nlohmann::json j;
        j["id"] = si.id;
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
        j["angle_ci"] = {si.ci.angle.lower, si.ci.angle.upper};
        j["offset_ci"] = {si.ci.offset.lower, si.ci.offset.upper};
        j["last_scanline_assignment"] = false;
        return j;
    }

    ScanlineInfo scanlineInfoFromJson(const nlohmann::json &j) {
        return ScanlineInfo{
            .id = j.at("id").get<uint32_t>(),
            .pointsCount = j.at("count"),
            .values = OffsetAngle{j.at("offset"), j.at("angle")},
            .ci = OffsetAngleMargin{
                .offset = RealMargin{j.at("offset_ci")[0], j.at("offset_ci")[1]},
                .angle = RealMargin{j.at("angle_ci")[0], j.at("angle_ci")[1]}
            },
            .theoreticalAngleBounds = ScanlineAngleBounds{
                .bottom = RealMargin{j.at("lower_min_theoretical_angle"), j.at("lower_max_theoretical_angle")},
                .top = RealMargin{j.at("upper_min_theoretical_angle"), j.at("upper_max_theoretical_angle")}
            },
            .uncertainty = j.at("uncertainty").is_null()
                               ? std::numeric_limits<double>::infinity()
                               : j.at("uncertainty").get<double>(),
            .houghVotes = j.at("hough_votes"),
            .houghHash = j.at("hash")
        };
    }

    nlohmann::json hashToConflictValueToJson(const HashToConflictValue &htc) {
        nlohmann::json j;
        j["conflictingScanlines"] = std::vector<uint32_t>(
            htc.conflictingScanlines.begin(), htc.conflictingScanlines.end()
        );
        j["votes"] = htc.votes;
        return j;
    }

    HashToConflictValue hashToConflictValueFromJson(const nlohmann::json &j) {
        std::unordered_set<uint32_t> scanlines;
        for (const auto &el: j.at("conflictingScanlines")) {
            scanlines.insert(el.get<uint32_t>());
        }

        return HashToConflictValue{
            .conflictingScanlines = std::move(scanlines),
            .votes = j.at("votes")
        };
    }

    nlohmann::json endReasonToJson(const EndReason er) {
        switch (er) {
            case EndReason::ALL_ASSIGNED:
                return "ALL_ASSIGNED";
            case EndReason::MAX_ITERATIONS:
                return "MAX_ITERATIONS";
            case EndReason::NO_MORE_PEAKS:
                return "NO_MORE_PEAKS";
            default:
                return "UNKNOWN";
        }
    }

    EndReason endReasonFromJson(const nlohmann::json &j) {
        const std::string reason = j.get<std::string>();

        if (reason == "ALL_ASSIGNED") {
            return EndReason::ALL_ASSIGNED;
        }
        if (reason == "MAX_ITERATIONS") {
            return EndReason::MAX_ITERATIONS;
        }
        if (reason == "NO_MORE_PEAKS") {
            return EndReason::NO_MORE_PEAKS;
        }

        throw std::invalid_argument("Unknown end reason: " + reason);
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

    VerticalIntrinsicsResult verticalIntrinsicsResultFromJson(const nlohmann::json &j) {
        VerticalIntrinsicsResult vir;
        vir.iterations = j.at("iterations");
        vir.scanlinesCount = j.at("scanlines_count");
        vir.endReason = endReasonFromJson(j.at("end_reason"));
        vir.unassignedPoints = j.at("unassigned_points");
        vir.pointsCount = j.at("points_count");
        for (const auto &entry: j.at("scanlines_attributes")) {
            vir.fullScanlines.scanlines.push_back(scanlineInfoFromJson(entry));
        }
        return vir;
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

    HoughCell houghCellFromJson(const nlohmann::json &j) {
        return HoughCell{
            .maxOffsetIndex = j.at("maxOffsetIndex"),
            .maxAngleIndex = j.at("maxAngleIndex"),
            .maxValues = offsetAngleFromJson(j.at("maxValues")),
            .votes = j.at("votes"),
            .hash = j.at("hash")
        };
    }

    nlohmann::json verticalBoundsToJson(const VerticalBounds &vb) {
        nlohmann::json j;
        j["phis"] = std::vector<double>(vb.phis.data(), vb.phis.data() + vb.phis.size());
        j["correction"] = std::vector<double>(vb.correction.data(), vb.correction.data() + vb.correction.size());
        j["final"] = std::vector<double>(vb.final.data(), vb.final.data() + vb.final.size());
        return j;
    }

    VerticalBounds verticalBoundsFromJson(const nlohmann::json &j) {
        return VerticalBounds{
            Eigen::Map<const Eigen::ArrayXd>(j.at("phis").get<std::vector<double>>().data(), j.at("phis").size()),
            Eigen::Map<const Eigen::ArrayXd>(
                j.at("correction").get<std::vector<double>>().data(), j.at("correction").size()
            ),
            Eigen::Map<const Eigen::ArrayXd>(j.at("final").get<std::vector<double>>().data(), j.at("final").size())
        };
    }

    nlohmann::json horizontalScanlineInfoToJson(const ScanlineHorizontalInfo &shi) {
        nlohmann::json j;
        j["resolution"] = shi.resolution;
        j["offset"] = shi.offset;
        j["thetaOffset"] = shi.thetaOffset;
        j["heuristic"] = shi.heuristic;
        return j;
    }

    ScanlineHorizontalInfo horizontalScanlineInfoFromJson(const nlohmann::json &j) {
        return ScanlineHorizontalInfo{
            .resolution = j.at("resolution"),
            .offset = j.at("offset"),
            .thetaOffset = j.at("thetaOffset"),
            .heuristic = j.at("heuristic")
        };
    }

    nlohmann::json horizontalIntrinsicsToJson(const HorizontalIntrinsicsResult &hir) {
        nlohmann::json j;
        j["scanlines_attributes"] = nlohmann::json::array();
        for (const auto &scanline: hir.scanlines) {
            j["scanlines_attributes"].push_back(horizontalScanlineInfoToJson(scanline));
        }
        return j;
    }

    HorizontalIntrinsicsResult horizontalIntrinsicsFromJson(const nlohmann::json &j) {
        HorizontalIntrinsicsResult hir;
        for (const auto &scanline: j.at("scanlines_attributes")) {
            hir.scanlines.push_back(horizontalScanlineInfoFromJson(scanline));
        }
        return hir;
    }

    nlohmann::json intrinsicsResultToJson(const IntrinsicsResult &i) {
        nlohmann::json j;
        j["vertical"] = verticalIntrinsicsResultToJson(i.vertical);
        j["horizontal"] = horizontalIntrinsicsToJson(i.horizontal);
        return j;
    }

    IntrinsicsResult intrinsicsResultFromJson(const nlohmann::json &j) {
        return IntrinsicsResult{
            .vertical = verticalIntrinsicsResultFromJson(j.at("vertical")),
            .horizontal = horizontalIntrinsicsFromJson(j.at("horizontal"))
        };
    }
} // accurate_ri
