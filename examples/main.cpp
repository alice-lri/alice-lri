#include <chrono>
#include <iostream>
#include <optional>
#include "accurate_ri/accurate_ri.hpp"
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

// TODO perf, identify memory  bottlenecks. Reuse Eigen buffers as much as possible, especially in loops
// TODO review each time I add to a collection whether I am copying or not. Especially maps/sets, use emplace
// TODO maybe every function that does not return an eigen pre-allocated buffer should take a parameter out instead
int main(int argc, char **argv) {
    std::string path;
    std::optional<int> accurateDigits = std::nullopt;
    std::optional<std::string> outputPath = std::nullopt;

    switch (argc) {
        case 1:
            std::cout << "Using default parameters" << std::endl;
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000019161.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000017276.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000013681.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000825.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000636.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0046_sync/velodyne_points/data/0000000124.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/0000004131.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0018_sync/velodyne_points/data/0000000183.bin";
            path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0018_sync/velodyne_points/data/0000000000.bin";
            accurateDigits = std::nullopt;
            outputPath = "../../Datasets/output/accurate_ri_cpp/";
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

    auto start = std::chrono::high_resolution_clock::now();
    accurate_ri::IntrinsicsResult result = accurate_ri::execute(points.x, points.y, points.z);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    return 0;
}
