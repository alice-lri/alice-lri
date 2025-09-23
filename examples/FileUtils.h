#pragma once
#include <optional>
#include <string>
#include "alice_lri/Core.hpp"

namespace FileUtils {
    struct Points {
        alice_lri::AliceArray<double> x, y, z;
    };

    Points loadBinaryFile(const std::string &filename);
}
