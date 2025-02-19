#pragma once
#include <string>
#include "accurate_ri.h"

namespace FileUtils {
    struct Points {
        std::vector<float> x, y, z;
    };

    Points loadBinaryFile(const std::string &filename);
}
