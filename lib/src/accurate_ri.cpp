#include "accurate_ri/accurate_ri.hpp"
#include "intrinsics/IntrinsicsEstimator.h"
#include "rangeimage/RangeImageUtils.h"
#include "utils/json/JsonConverters.h"
#include "utils/logger/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {

    Intrinsics train(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray) {
        PROFILE_SCOPE("TOTAL");

        if (xArray.size() == 0 || yArray.size() == 0 || zArray.size() == 0) {
            return Intrinsics(0);
        }

        const PointArray points(xArray, yArray, zArray);
        return IntrinsicsEstimator::estimate(points);
    }

    Intrinsics train(const PointCloud::Float &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(points.x.data(), size).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(points.y.data(), size).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(points.z.data(), size).cast<double>();

        const Intrinsics result = train(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    Intrinsics train(const PointCloud::Double &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(points.x.data(), size);
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(points.y.data(), size);
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(points.z.data(), size);

        const Intrinsics result = train(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    DebugIntrinsics debugTrain(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray) {
        PROFILE_SCOPE("TOTAL");

        if (xArray.size() == 0 || yArray.size() == 0 || zArray.size() == 0) {
            return DebugIntrinsics(0);
        }

        const PointArray points(xArray, yArray, zArray);
        return IntrinsicsEstimator::debugEstimate(points);
    }

    DebugIntrinsics debugTrain(const PointCloud::Float &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(points.x.data(), size).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(points.y.data(), size).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(points.z.data(), size).cast<double>();

        const DebugIntrinsics result = debugTrain(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    DebugIntrinsics debugTrain(const PointCloud::Double &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(points.x.data(), size);
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(points.y.data(), size);
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(points.z.data(), size);

        const DebugIntrinsics result = debugTrain(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(points.x.data(), size).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(points.y.data(), size).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(points.z.data(), size).cast<double>();

        return RangeImageUtils::computeRangeImage(intrinsics, xArray, yArray, zArray);
    }

    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points) {
        const auto size = static_cast<Eigen::Index>(points.x.size());

        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(points.x.data(), size);
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(points.y.data(), size);
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(points.z.data(), size);

        return RangeImageUtils::computeRangeImage(intrinsics, xArray, yArray, zArray);
    }

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &rangeImage) {
        return RangeImageUtils::unProjectRangeImage(intrinsics, rangeImage);
    }

    Intrinsics intrinsicsFromJsonStr(const AliceString &json) {
        return intrinsicsFromJson(json);
    }

    AliceString intrinsicsToJsonStr(const Intrinsics &result) {
        const std::string json = intrinsicsToJson(result);
        return AliceString(json.c_str());
    }

    Intrinsics intrinsicsFromJsonFile(const char *path) {
        std::ifstream inFile(path);
        nlohmann::json json;
        inFile >> json;
        return intrinsicsFromJson(json);
    }

    void intrinsicsToJsonFile(const Intrinsics &result, const char *outputPath) {
        const nlohmann::json json = intrinsicsToJson(result);
        std::ofstream outFile(outputPath);
        outFile << json.dump(4);
    }
}
