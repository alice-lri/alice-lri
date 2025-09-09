#pragma once
#include "accurate_ri/public_structs.hpp"

// TODO as json str methods
namespace accurate_ri {
    ACCURATE_RI_API Intrinsics train(const PointCloud::Float &points);

    ACCURATE_RI_API Intrinsics train(const PointCloud::Double &points);

    ACCURATE_RI_API Intrinsics readFromJson(const char *path);

    ACCURATE_RI_API void writeToJson(const Intrinsics &result, const char *outputPath);

    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points);

    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points);

    ACCURATE_RI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage);
}
