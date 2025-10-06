#pragma once
#include "util/AliceString.hpp"
#include "alice_lri/Structs.hpp"
#include "alice_lri/Result.hpp"

namespace alice_lri {
    ALICE_LRI_API Result<Intrinsics> estimateIntrinsics(const PointCloud::Float &points) noexcept;
    ALICE_LRI_API Result<Intrinsics> estimateIntrinsics(const PointCloud::Double &points) noexcept;
    ALICE_LRI_API Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Float &points) noexcept;
    ALICE_LRI_API Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Double &points) noexcept;

    ALICE_LRI_API Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) noexcept;
    ALICE_LRI_API Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) noexcept;
    ALICE_LRI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept;

    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonStr(const AliceString &json) noexcept;
    ALICE_LRI_API AliceString intrinsicsToJsonStr(const Intrinsics &result, int32_t indent = -1) noexcept;
    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonFile(const char *path) noexcept;
    ALICE_LRI_API Status intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath, int32_t indent = -1) noexcept;
}
