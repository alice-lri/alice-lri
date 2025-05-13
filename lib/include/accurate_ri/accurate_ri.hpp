#pragma once
#include <string>
#include <vector>
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    IntrinsicsResult execute(const PointCloud::Float &points);

    IntrinsicsResult execute(const PointCloud::Double &points);

    void writeToJson(const IntrinsicsResult &result, const std::string &outputPath);

    RangeImage projectToRangeImage(const IntrinsicsResult &intrinsics, const PointCloud::Float &points);

    RangeImage projectToRangeImage(const IntrinsicsResult &intrinsics, const PointCloud::Double &points);

    PointCloud::Double unProjectToPointCloud(const IntrinsicsResult &intrinsics, const RangeImage &rangeImage);
}
