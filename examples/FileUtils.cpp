#include "FileUtils.h"

#include <cstdint>
#include <fstream>

FileUtils::Points FileUtils::loadBinaryFile(const std::string &filename) {
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
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        throw std::runtime_error("Error reading file");
    }

    size_t pointsCount = buffer.size() / 4;

    Points result;
    result.x.reserve(pointsCount);
    result.y.reserve(pointsCount);
    result.z.reserve(pointsCount);

    for (uint32_t i = 0; i < pointsCount; ++i) {
        result.x[i] = buffer[i * 4];
        result.y[i] = buffer[i * 4 + 1];
        result.z[i] = buffer[i * 4 + 2];
    }

    return result;
}
