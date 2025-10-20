#include "FileUtils.h"

#include <cstdint>
#include <fstream>
#include <optional>
#include <cmath>
#include <vector>

namespace FileUtils {

    Points loadBinaryFile(const std::string &filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        file.seekg(0, std::ios::end);
        std::streamsize fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        if (fileSize % sizeof(float) != 0) {
            throw std::runtime_error("File size is not a multiple of float size.");
        }

        std::vector<float> buffer(fileSize / sizeof(float));
        if (!file.read(reinterpret_cast<char *>(buffer.data()), fileSize)) {
            throw std::runtime_error("Error reading file");
        }

        size_t pointsCount = buffer.size() / 4;

        Points result;
        result.x.reserve(pointsCount);
        result.y.reserve(pointsCount);
        result.z.reserve(pointsCount);

        for (uint32_t i = 0; i < pointsCount; ++i) {
            const double x = buffer[i * 4];
            const double y = buffer[i * 4 + 1];
            const double z = buffer[i * 4 + 2];

            if (x == 0 && y == 0 && z == 0) {
                continue;
            }

            result.x.emplace_back(x);
            result.y.emplace_back(y);
            result.z.emplace_back(z);
        }

        result.x.shrink_to_fit();
        result.y.shrink_to_fit();
        result.z.shrink_to_fit();

        return result;
    }
}
