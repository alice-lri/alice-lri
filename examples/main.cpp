#include "accurate_ri.h"
#include "FileUtils.h"

int main() {
    FileUtils::Points points = FileUtils::loadBinaryFile("../../Datasets/LiDAR/kitti_organized/Organized/road/2011_10_03_drive_0042/data/0000000000.bin");
    accurate_ri::execute(points.x, points.y, points.z);
}
