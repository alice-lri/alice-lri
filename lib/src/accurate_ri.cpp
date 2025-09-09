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
            return {};
        }

        const PointArray points(xArray, yArray, zArray);
        IntrinsicsEstimator estimator{};

        return estimator.estimate(points);
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

    Intrinsics readFromJson(const char *path) {
        std::ifstream inFile(path);
        nlohmann::json json;
        inFile >> json;
        return intrinsicsResultFromJson(json);
    }

    void writeToJson(const Intrinsics &result, const char *outputPath) {
        nlohmann::json json = intrinsicsResultToJson(result);
        std::ofstream outFile(outputPath);
        outFile << json.dump(4);
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
}
