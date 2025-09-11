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

// TODO perf, identify memory  bottlenecks. Reuse Eigen buffers as much as possible, especially in loops
// TODO review each time I add to a collection whether I am copying or not. Especially maps/sets, use emplace
// TODO maybe every function that does not return an eigen pre-allocated buffer should take a parameter out instead
// TODO handle receiving points like all zeros and stuff like that
// TODO decide what to do with logs and stuff for the library version
// TODO maybe reserve log warn and error for user relevant logs
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
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210716/ouster_points/data/0000041266.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000000024.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211208/ouster_points/data/0000018613.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000013963.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211012/ouster_points/data/0000010410.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210716/ouster_points/data/0000039648.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000017973.bin";
            // path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000018973.bin";
            path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000825.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000636.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0046_sync/velodyne_points/data/0000000124.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/0000004131.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0018_sync/velodyne_points/data/0000000183.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0018_sync/velodyne_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0086_sync/velodyne_points/data/0000000386.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0086_sync/velodyne_points/data/0000000421.bin";
            //path = "../../Datasets/LiDAR/kitti/2011_09_28/2011_09_28_drive_0205_sync/velodyne_points/data/0000000000.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000319.bin";
            //path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000352.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0057_sync/velodyne_points/data/0000000293.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0093_sync/velodyne_points/data/0000000217.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/0000000465.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0079_sync/velodyne_points/data/0000000099.bin";
            // path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0087_sync/velodyne_points/data/0000000263.bin";
            accurateDigits = std::nullopt;
            outputPath = "../../Datasets/output/accurate_ri_cpp/intrinsics.json";
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

    if (accurateDigits == -1) {
        accurateDigits = std::nullopt;
    }

    FileUtils::Points points = FileUtils::loadBinaryFile(path, accurateDigits);

    double initialZ = points.z[12];
    auto start = std::chrono::high_resolution_clock::now();

    const accurate_ri::PointCloud::Double cloud(std::move(points.x), std::move(points.y), std::move(points.z));

    accurate_ri::Intrinsics intrinsics = accurate_ri::train(cloud);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    if (outputPath) {
        accurate_ri::intrinsicsToJsonFile(intrinsics, outputPath->data());
    }

    const accurate_ri::RangeImage ri = accurate_ri::projectToRangeImage(intrinsics, cloud);
    accurate_ri::unProjectToPointCloud(intrinsics, ri);

    return 0;
}
