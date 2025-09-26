#include <chrono>
#include <iostream>
#include <optional>
#include "alice_lri/Core.hpp"
#include "FileUtils.h"

int main(int argc, char **argv) {
    FileUtils::Points points = FileUtils::loadBinaryFile("resources/kitti_frame.bin");
    const alice_lri::PointCloud::Double cloud(std::move(points.x), std::move(points.y), std::move(points.z));
    const alice_lri::Result<alice_lri::Intrinsics> intrinsics = alice_lri::estimateIntrinsics(cloud);

    if (!intrinsics) {
        std::cerr << intrinsics.status().message.c_str();
        return 1;
    }

    const auto status = alice_lri::intrinsicsToJsonFile(*intrinsics, "intrinsics.json", 4);
    if (!status) {
        std::cerr << status.message.c_str();
        return 1;
    } else {
        std::cout << "Generated intrinsics.json file" << std::endl;
    }

    const auto ri = alice_lri::projectToRangeImage(*intrinsics, cloud);
    if (!ri) {
        std::cerr << ri.status().message.c_str();
        return 1;
    }

    alice_lri::unProjectToPointCloud(*intrinsics, *ri);

    return 0;
}
