#include <chrono>
#include <iostream>
#include <optional>
#include "alice_lri/Core.hpp"
#include "utils/FileUtils.h"

int main(int argc, char **argv) {
    std::optional<std::string> outputPath = std::nullopt;
    std::string path = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211208/ouster_points/data/0000020365.bin";
    // std::string path = "../../Datasets/LiDAR/kitti/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin";

    FileUtils::Points points = FileUtils::loadBinaryFile(path);

    auto start = std::chrono::high_resolution_clock::now();

    const alice_lri::PointCloud::Double cloud(std::move(points.x), std::move(points.y), std::move(points.z));
    const alice_lri::Result<alice_lri::Intrinsics> intrinsics = alice_lri::estimateIntrinsics(cloud);

    if (!intrinsics) {
        std::cerr << intrinsics.status().message.c_str();
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    if (outputPath) {
        const auto status = alice_lri::intrinsicsToJsonFile(*intrinsics, outputPath->data(), 4);
        if (!status) {
            std::cerr << status.message.c_str();
            return 1;
        }
    }

    start = std::chrono::high_resolution_clock::now();
    const auto ri = alice_lri::projectToRangeImage(*intrinsics, cloud);
    if (!ri) {
        std::cerr << ri.status().message.c_str();
        return 1;
    }

    end = std::chrono::high_resolution_clock::now();
    std::cout << "Project time: " << std::chrono::duration<double>(end - start).count() << "s" << std::endl;

    alice_lri::unProjectToPointCloud(*intrinsics, *ri);

    return 0;
}
