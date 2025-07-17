#pragma once
#include "accurate_ri/public_structs.hpp"

// TODO as json str methods
namespace accurate_ri {
    ACCURATE_RI_API IntrinsicsResult train(const PointCloud::Float &points);

    ACCURATE_RI_API IntrinsicsResult train(const PointCloud::Double &points);

    ACCURATE_RI_API IntrinsicsResult readFromJson(const char *path);

    ACCURATE_RI_API void writeToJson(const IntrinsicsResult &result, const char *outputPath);

    ACCURATE_RI_API RangeImage projectToRangeImage(const IntrinsicsResult &intrinsics, const PointCloud::Float &points);

    ACCURATE_RI_API RangeImage projectToRangeImage(const IntrinsicsResult &intrinsics, const PointCloud::Double &points);

    ACCURATE_RI_API PointCloud::Double unProjectToPointCloud(const IntrinsicsResult &intrinsics, const RangeImage &rangeImage);
}
