#pragma once
#include <nlohmann/json.hpp>
#include "alice_lri/public_structs.hpp"

namespace alice_lri {
    nlohmann::json scanlineToJson(const Scanline& scanline);
    Scanline scanlineFromJson(const nlohmann::json &j);

    nlohmann::json intrinsicsToJson(const Intrinsics &intrinsics);
    Intrinsics intrinsicsFromJson(const nlohmann::json &j);
}
