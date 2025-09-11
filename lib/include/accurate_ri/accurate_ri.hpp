#pragma once
#include "accurate_ri/AliceString.h"
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    ACCURATE_RI_API Intrinsics train(const PointCloud::Float &points);
    ACCURATE_RI_API Intrinsics train(const PointCloud::Double &points);
    ACCURATE_RI_API DebugIntrinsics debugTrain(const PointCloud::Float &points);
    ACCURATE_RI_API DebugIntrinsics debugTrain(const PointCloud::Double &points);

    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points);
    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points);
    ACCURATE_RI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage);

    ACCURATE_RI_API Intrinsics intrinsicsFromJsonStr(const AliceString& json);
    ACCURATE_RI_API AliceString intrinsicsToJsonStr(const Intrinsics &result);
    ACCURATE_RI_API Intrinsics intrinsicsFromJsonFile(const char *path);
    ACCURATE_RI_API void intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath);
}