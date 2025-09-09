#pragma once
#include <nlohmann/json.hpp>
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    nlohmann::json scanlineToJson(const Scanline& scanline);
    Scanline scanlineFromJson(const nlohmann::json &j);

    nlohmann::json intrinsicsToJson(const Intrinsics &i);
    Intrinsics intrinsicsFromJson(const nlohmann::json &j);
}
