#include <iostream>
#include <optional>
#include "accurate_ri.h"
#include "FileUtils.h"

std::optional<int> secureStoi(const std::string &str) {
    try {
        return std::stoi(str);
    } catch (const std::exception &e) {
        return std::nullopt;
    }
}

// TODO this is a temporary hack to compare traces, remove
void setCloudPath(const std::string &path) {
    const std::string from = "../../Datasets/LiDAR";
    const std::string to = "../../datasets";
    std::string newPath = path;

    newPath.replace(newPath.find(from), from.length(), to);
    accurate_ri::setCloudPath(newPath);
}

int main(int argc, char **argv) {
    std::string path;
    std::optional<int> accurateDigits = std::nullopt;
    std::optional<std::string> outputPath = std::nullopt;

    switch (argc) {
        case 1:
            std::cout << "Using default parameters" << std::endl;
            path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000009025.bin";
            accurateDigits = std::nullopt;
            outputPath = "output/";
            break;
        case 2:
            path = argv[1];
            break;
        case 3:
            path = argv[1];
            accurateDigits = secureStoi(argv[2]);
            break;
        case 4:
            path = argv[1];
            accurateDigits = secureStoi(argv[2]);
            outputPath = argv[3];
            break;
        default:
            std::cout << "Usage: " << argv[0] << " [path] [accurateDigits] [outputPath]" << std::endl;
            return 1;
    }

    FileUtils::Points points = FileUtils::loadBinaryFile(path, accurateDigits);
    setCloudPath(path);
    accurate_ri::setOutputPath(outputPath);

    accurate_ri::execute(points.x, points.y, points.z);
    return 0;
}
