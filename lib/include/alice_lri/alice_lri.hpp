#pragma once
#include "alice_lri/AliceString.h"
#include "alice_lri/public_structs.hpp"
#include "alice_lri/Result.h"

namespace alice_lri {
    ALICE_LRI_API Result<Intrinsics> train(const PointCloud::Float &points) noexcept;
    ALICE_LRI_API Result<Intrinsics> train(const PointCloud::Double &points) noexcept;
    ALICE_LRI_API Result<DebugIntrinsics> debugTrain(const PointCloud::Float &points) noexcept;
    ALICE_LRI_API Result<DebugIntrinsics> debugTrain(const PointCloud::Double &points) noexcept;

    ALICE_LRI_API Result<RangeImage> projectToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Float &points
    ) noexcept;
    ALICE_LRI_API Result<RangeImage> projectToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Double &points
    ) noexcept;
    ALICE_LRI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept;

    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonStr(const AliceString &json) noexcept;
    ALICE_LRI_API AliceString intrinsicsToJsonStr(const Intrinsics &result, int32_t indent = -1) noexcept;
    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonFile(const char *path) noexcept;
    ALICE_LRI_API Status intrinsicsToJsonFile(
        const Intrinsics &result, const char *outputPath, int32_t indent = -1
    ) noexcept;
}
