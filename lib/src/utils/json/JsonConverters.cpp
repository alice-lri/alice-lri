#include "JsonConverters.h"

namespace alice_lri {
    nlohmann::json scanlineToJson(const Scanline& scanline) {
        nlohmann::json j;
        j["verticalOffset"] = scanline.verticalOffset;
        j["verticalAngle"] = scanline.verticalAngle;
        j["horizontalOffset"] = scanline.horizontalOffset;
        j["azimuthalOffset"] = scanline.azimuthalOffset;
        j["resolution"] = scanline.resolution;

        return j;
    }

    Scanline scanlineFromJson(const nlohmann::json &j) {
        return Scanline{
            .verticalOffset = j["verticalOffset"],
            .verticalAngle = j["verticalAngle"],
            .horizontalOffset = j["horizontalOffset"],
            .azimuthalOffset = j["azimuthalOffset"],
            .resolution = j["resolution"]
        };
    }

    nlohmann::json intrinsicsToJson(const Intrinsics &intrinsics) {
        nlohmann::json j;
        j["scanlines"] = nlohmann::json::array();

        for (int i = 0; i < intrinsics.scanlines.size(); ++i) {
            j["scanlines"].push_back(scanlineToJson(intrinsics.scanlines[i]));
        }

        return j;
    }

    Intrinsics intrinsicsFromJson(const nlohmann::json &j) {
        Intrinsics intrinsics(j.at("scanlines").size());

        for (int i = 0; i < intrinsics.scanlines.size(); ++i) {
            intrinsics.scanlines[i] = scanlineFromJson(j.at("scanlines")[i]);
        }

        return intrinsics;
    }
}
