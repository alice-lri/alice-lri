#include "accurate_ri/accurate_ri.hpp"

#include "accurate_ri/Result.h"
#include "intrinsics/IntrinsicsEstimator.h"
#include "rangeimage/RangeImageUtils.h"
#include "utils/json/JsonConverters.h"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {
    template <typename Scalar>
    Result<bool> validateInput(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        const bool equalSizes = x.size() == y.size() && y.size() == z.size();
        if (!equalSizes) {
            return Result<bool>(Status::error(ErrorCode::MISMATCHED_SIZES));
        }

        if (x.empty() || y.empty() || z.empty()) {
            return Result<bool>(Status::error(ErrorCode::EMPTY_POINT_CLOUD));
        }

        const Eigen::ArrayXd xCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(x.data(), x.size()).template cast<double>();
        const Eigen::ArrayXd yCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(y.data(), y.size()).template cast<double>();
        const Eigen::ArrayXd zCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(z.data(), z.size()).template cast<double>();

        if ((xCast.square() + yCast.square()).minCoeff() <= 0) {
            return Result<bool>(Status::error(ErrorCode::RANGES_XY_ZERO));
        }

        return Result(true);
    }

    template <typename Scalar>
    Result<PointArray> validateAndBuildPointArray(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        const auto validation = validateInput(x, y, z);
        if (!validation) {
            return Result<PointArray>(validation.status());
        }

        const Eigen::ArrayXd xCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(x.data(), x.size()).template cast<double>();
        const Eigen::ArrayXd yCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(y.data(), y.size()).template cast<double>();
        const Eigen::ArrayXd zCast = Eigen::Map<const Eigen::ArrayX<Scalar>>(z.data(), z.size()).template cast<double>();

        return Result(PointArray(xCast, yCast, zCast));
    }

    template <typename Scalar>
    Result<Intrinsics> train(
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
            return Result<Intrinsics>(Status::error(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    template <typename Scalar>
    Result<DebugIntrinsics> debugTrain(
        const AliceArray<Scalar> &x, const AliceArray<Scalar> &y, const AliceArray<Scalar> &z
    ) noexcept {
        try {
            PROFILE_SCOPE("TOTAL");
            const Result<PointArray> pointsResult = validateAndBuildPointArray(x, y, z);

            if (!pointsResult) {
                return Result<DebugIntrinsics>(pointsResult.status());
            }

            return Result(IntrinsicsEstimator::debugEstimate(*pointsResult));
        } catch (const std::exception &e) {
            return Result<DebugIntrinsics>(Status::error(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    Result<Intrinsics> train(const PointCloud::Float &points) noexcept {
        const auto result = train(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<Intrinsics> train(const PointCloud::Double &points) noexcept {
        const auto result = train(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<DebugIntrinsics> debugTrain(const PointCloud::Float &points) noexcept {
        const auto result = debugTrain(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<DebugIntrinsics> debugTrain(const PointCloud::Double &points) noexcept {
        const auto result = debugTrain(points.x, points.y, points.z);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) noexcept {
        try {
            const auto validation = validateInput(points.x, points.y, points.z);
            if (!validation) {
                return Result<RangeImage>(validation.status());
            }

            return Result(RangeImageUtils::projectToRangeImage(intrinsics, points));
        } catch (const std::exception &e) {
            return Result<RangeImage>(Status::error(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    Result<RangeImage> projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) noexcept {
        try {
            const auto validation = validateInput(points.x, points.y, points.z);
            if (!validation) {
                return Result<RangeImage>(validation.status());
            }

            return Result(RangeImageUtils::projectToRangeImage(intrinsics, points));
        } catch (const std::exception &e) {
            return Result<RangeImage>(Status::error(ErrorCode::INTERNAL_ERROR, AliceString(e.what())));
        }
    }

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) noexcept {
        return RangeImageUtils::unProjectToPointCloud(intrinsics, rangeImage);
    }

    // TODO handle exceptions and so on
    Intrinsics intrinsicsFromJsonStr(const AliceString &json) noexcept {
        return intrinsicsFromJson(json);
    }

    AliceString intrinsicsToJsonStr(const Intrinsics &result, const int32_t indent) noexcept {
        const std::string json = intrinsicsToJson(result).dump(indent);
        return AliceString(json.c_str());
    }

    Intrinsics intrinsicsFromJsonFile(const char *path) noexcept {
        std::ifstream inFile(path);
        nlohmann::json json;
        inFile >> json;
        return intrinsicsFromJson(json);
    }

    void intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath, const int32_t indent) noexcept {
        const nlohmann::json json = intrinsicsToJson(result);
        std::ofstream outFile(outputPath);
        outFile << json.dump(indent);
    }
}
