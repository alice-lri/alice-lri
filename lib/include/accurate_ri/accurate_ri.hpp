#pragma once
#include "accurate_ri/AliceString.h"
#include "accurate_ri/public_structs.hpp"
#include "accurate_ri/Result.h"

namespace accurate_ri {
    ACCURATE_RI_API Result<Intrinsics> train(const PointCloud::Float &points) noexcept;
    ACCURATE_RI_API Result<Intrinsics> train(const PointCloud::Double &points) noexcept;
    ACCURATE_RI_API Result<DebugIntrinsics> debugTrain(const PointCloud::Float &points) noexcept;
    ACCURATE_RI_API Result<DebugIntrinsics> debugTrain(const PointCloud::Double &points) noexcept;

    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) noexcept;
    ACCURATE_RI_API RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) noexcept;
    ACCURATE_RI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept;

    ACCURATE_RI_API Intrinsics intrinsicsFromJsonStr(const AliceString& json) noexcept;
    ACCURATE_RI_API AliceString intrinsicsToJsonStr(const Intrinsics &result, int32_t indent = -1) noexcept;
    ACCURATE_RI_API Intrinsics intrinsicsFromJsonFile(const char *path) noexcept;
    ACCURATE_RI_API void intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath, int32_t indent = -1) noexcept;
}
