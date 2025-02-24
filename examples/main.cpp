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
    const std::string path = "../../Datasets/LiDAR/kitti_organized/Organized/road/2011_10_03_drive_0042/data/0000000000.bin";
    FileUtils::Points points = FileUtils::loadBinaryFile(path);

    setCloudPath(path);
    accurate_ri::execute(points.x, points.y, points.z);
}
