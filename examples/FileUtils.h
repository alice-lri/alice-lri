#pragma once
#include <optional>
#include <string>
#include "accurate_ri/accurate_ri.hpp"

namespace FileUtils {
    struct Points {
        accurate_ri::AliceArray<double> x, y, z;
    };

    Points loadBinaryFile(const std::string &filename, const std::optional<int>& accurateDigits);
}
