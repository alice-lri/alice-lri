/**
 * @file Core.hpp
 * @brief Core definitions and main classes for Alice LRI library.
 */
#pragma once
#include "util/AliceString.hpp"
#include "alice_lri/Structs.hpp"
#include "alice_lri/Result.hpp"

namespace alice_lri {

    /**
     * @brief Estimate sensor intrinsics from a float point cloud.
     * @param points Input point cloud (float precision).
     * @return Result containing estimated Intrinsics or error status.
     */
    ALICE_LRI_API Result<Intrinsics> estimateIntrinsics(const PointCloud::Float &points) noexcept;

    /**
     * @brief Estimate sensor intrinsics from a double point cloud.
     * @param points Input point cloud (double precision).
     * @return Result containing estimated Intrinsics or error status.
     */
    ALICE_LRI_API Result<Intrinsics> estimateIntrinsics(const PointCloud::Double &points) noexcept;

    /**
     * @brief Estimate detailed sensor intrinsics from a float point cloud.
     * @param points Input point cloud (float precision).
     * @return Result containing detailed Intrinsics or error status.
     */
    ALICE_LRI_API Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Float &points) noexcept;

    /**
     * @brief Estimate detailed sensor intrinsics from a double point cloud.
     * @param points Input point cloud (double precision).
     * @return Result containing detailed Intrinsics or error status.
     */
    ALICE_LRI_API Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Double &points) noexcept;

    /**
     * @brief Project a point cloud to a range image using given intrinsics (float).
     * @param intrinsics Sensor intrinsics.
     * @param points Input point cloud (float precision).
     * @return Result containing RangeImage or error status.
     */
    ALICE_LRI_API Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) noexcept;

    /**
     * @brief Project a point cloud to a range image using given intrinsics (double).
     * @param intrinsics Sensor intrinsics.
     * @param points Input point cloud (double precision).
     * @return Result containing RangeImage or error status.
     */
    ALICE_LRI_API Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) noexcept;

    /**
     * @brief Unproject a range image to a double point cloud using given intrinsics.
     * @param intrinsics Sensor intrinsics.
     * @param rangeImage Input range image.
     * @return Unprojected point cloud (double precision).
     */
    ALICE_LRI_API PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept;

    /**
     * @brief Parse Intrinsics from a JSON string.
     * @param json JSON string.
     * @return Result containing Intrinsics or error status.
     */
    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonStr(const AliceString &json) noexcept;

    /**
     * @brief Serialize Intrinsics to a JSON string.
     * @param result Intrinsics to serialize.
     * @param indent Indentation for pretty printing (-1 for compact).
     * @return JSON string.
     */
    ALICE_LRI_API AliceString intrinsicsToJsonStr(const Intrinsics &result, int32_t indent = -1) noexcept;

    /**
     * @brief Parse Intrinsics from a JSON file.
     * @param path Path to JSON file.
     * @return Result containing Intrinsics or error status.
     */
    ALICE_LRI_API Result<Intrinsics> intrinsicsFromJsonFile(const char *path) noexcept;

    /**
     * @brief Write Intrinsics to a JSON file.
     * @param result Intrinsics to write.
     * @param outputPath Output file path.
     * @param indent Indentation for pretty printing (-1 for compact).
     * @return Status of the operation.
     */
    ALICE_LRI_API Status intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath, int32_t indent = -1) noexcept;
}
