#include <optional>
#include "accurate_ri.h"
#include "FileUtils.h"

// TODO this is a temporary hack to compare traces, remove
void setCloudPath(const std::string& path) {
    const std::string from = "../../Datasets/LiDAR";
    const std::string to = "../../datasets";
    std::string newPath = path;

    newPath.replace(newPath.find(from), from.length(), to);
    accurate_ri::setCloudPath(newPath);
}

int main() {
    const std::string path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000009025.bin";
    const std::optional<int> accurateDigits = 4;
    FileUtils::Points points = FileUtils::loadBinaryFile(path, accurateDigits);

    setCloudPath(path);
    accurate_ri::execute(points.x, points.y, points.z);
}
