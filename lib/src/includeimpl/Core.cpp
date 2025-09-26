#include "alice_lri/Core.hpp"

#include "alice_lri/Result.hpp"
#include "intrinsics/IntrinsicsEstimator.h"
#include "rangeimage/RangeImageUtils.h"
#include "utils/json/JsonConverters.h"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"

namespace alice_lri {
    template <typename Scalar>
    Status validateInput(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        const bool equalSizes = x.size() == y.size() && y.size() == z.size();
        if (!equalSizes) {
            return Status::buildError(ErrorCode::MISMATCHED_SIZES);
        }

        if (x.empty() || y.empty() || z.empty()) {
            return Status::buildError(ErrorCode::EMPTY_POINT_CLOUD);
        }

        const Eigen::ArrayXd xCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(x.data(), x.size()).template cast<double>();
        const Eigen::ArrayXd yCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(y.data(), y.size()).template cast<double>();
        const Eigen::ArrayXd zCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(z.data(), z.size()).template cast<double>();

        if ((xCast.square() + yCast.square()).minCoeff() <= 0) {
            return Status::buildError(ErrorCode::RANGES_XY_ZERO);
        }

        return Status::buildOk();
    }

    template <typename Scalar>
    Result<PointArray> validateAndBuildPointArray(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        const auto validationStatus = validateInput(x, y, z);
        if (!validationStatus) {
            return Result<PointArray>(validationStatus);
        }

        const Eigen::ArrayXd xCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(x.data(), x.size()).template cast<double>();
        const Eigen::ArrayXd yCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(y.data(), y.size()).template cast<double>();
        const Eigen::ArrayXd zCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(z.data(), z.size()).template cast<double>();

        return Result(PointArray(xCast, yCast, zCast));
    }

    template <typename Scalar>
    Result<Intrinsics> estimateIntrinsics(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        PROFILE_SCOPE("TOTAL");
        try {
            const Result<PointArray> pointsResult = validateAndBuildPointArray(x, y, z);

            if (!pointsResult) {
                return Result<Intrinsics>(pointsResult.status());
            }

            return Result(IntrinsicsEstimator::estimate(*pointsResult));
        } catch (const std::exception &e) {
            return Result<Intrinsics>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    template <typename Scalar>
    Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        try {
            PROFILE_SCOPE("TOTAL");
            const Result<PointArray> pointsResult = validateAndBuildPointArray(x, y, z);

            if (!pointsResult) {
                return Result<IntrinsicsDetailed>(pointsResult.status());
            }

            return Result(IntrinsicsEstimator::debugEstimate(*pointsResult));
        } catch (const std::exception &e) {
            return Result<IntrinsicsDetailed>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    Result<Intrinsics> estimateIntrinsics(const PointCloud::Float &points) noexcept {
        const auto result = estimateIntrinsics(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<Intrinsics> estimateIntrinsics(const PointCloud::Double &points) noexcept {
        const auto result = estimateIntrinsics(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Float &points) noexcept {
        const auto result = estimateIntrinsicsDetailed(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<IntrinsicsDetailed> estimateIntrinsicsDetailed(const PointCloud::Double &points) noexcept {
        const auto result = estimateIntrinsicsDetailed(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) noexcept {
        try {
            const auto validationStatus = validateInput(points.x, points.y, points.z);
            if (!validationStatus) {
                return Result<RangeImage>(validationStatus);
            }

            return Result(RangeImageUtils::projectToRangeImage(intrinsics, points));
        } catch (const std::exception &e) {
            return Result<RangeImage>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) noexcept {
        try {
            const auto validationStatus = validateInput(points.x, points.y, points.z);
            if (!validationStatus) {
                return Result<RangeImage>(validationStatus);
            }

            return Result(RangeImageUtils::projectToRangeImage(intrinsics, points));
        } catch (const std::exception &e) {
            return Result<RangeImage>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept {
        return RangeImageUtils::unProjectToPointCloud(intrinsics, rangeImage);
    }

    Result<Intrinsics> intrinsicsFromJsonStr(const AliceString &json) noexcept {
        try {
            return Result(intrinsicsFromJson(json));
        } catch (const std::exception &e) {
            return Result<Intrinsics>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    AliceString intrinsicsToJsonStr(const Intrinsics &result, const int32_t indent) noexcept {
        const std::string json = intrinsicsToJson(result).dump(indent);
        return AliceString(json.c_str());
    }

    Result<Intrinsics> intrinsicsFromJsonFile(const char *path) noexcept {
        try {
            std::ifstream inFile;
            inFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            inFile.open(path, std::ifstream::in);

            nlohmann::json json;
            inFile >> json;

            return Result(intrinsicsFromJson(json));
        } catch (const std::exception &e) {
            return Result<Intrinsics>(Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    Status intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath, const int32_t indent) noexcept {
        try {
            const nlohmann::json json = intrinsicsToJson(result);
            std::ofstream outFile;

            outFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
            outFile.open(outputPath, std::ofstream::out | std::ofstream::trunc);
            outFile << json.dump(indent);

            return Status::buildOk();
        } catch (const std::exception &e) {
            return Status::buildError(ErrorCode::INTERNAL_ERROR, AliceString(e.what()));
        }
    }
}
