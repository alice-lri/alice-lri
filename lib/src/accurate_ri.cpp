#include "accurate_ri/accurate_ri.hpp"
#include "intrinsics/IntrinsicsEstimator.h"
#include "rangeimage/RangeImageUtils.h"
#include "utils/json/JsonConverters.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {
    // TODO remove this, the library should be path agnostic, just here for the trace file
    std::string cloudPath;

    void hello() {
        LOG_INFO("Hello");
        LOG_DEBUG("Hello2");
    }

    IntrinsicsResult execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(x.data(), x.size()).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(y.data(), y.size()).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(z.data(), z.size()).cast<double>();

        const IntrinsicsResult result = execute(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    IntrinsicsResult execute(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(x.data(), x.size());
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(y.data(), y.size());
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(z.data(), z.size());

        const IntrinsicsResult result = execute(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();

        return result;
    }

    IntrinsicsResult execute(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray) {
        PROFILE_SCOPE("TOTAL");

        if (xArray.size() == 0 || yArray.size() == 0 || zArray.size() == 0) {
            return IntrinsicsResult();
        }

        const PointArray points(xArray, yArray, zArray);
        IntrinsicsEstimator estimator = IntrinsicsEstimator();

        return estimator.estimate(points);
    }

    void writeToJson(const IntrinsicsResult &result, const std::string &outputPath) {
        nlohmann::json json = intrinsicsResultToJson(result);
        std::ofstream outFile(outputPath);
        outFile << json.dump(4);
    }

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const std::vector<float> &x, const std::vector<float> &y,
        const std::vector<float> &z
    ) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(x.data(), x.size()).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(y.data(), y.size()).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(z.data(), z.size()).cast<double>();

        return computeRangeImage(intrinsics, xArray, yArray, zArray);
    }

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const std::vector<double> &x, const std::vector<double> &y,
        const std::vector<double> &z
    ) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(x.data(), x.size());
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(y.data(), y.size());
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(z.data(), z.size());

        return computeRangeImage(intrinsics, xArray, yArray, zArray);
    }

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray,
        const Eigen::ArrayXd &zArray
    ) {
        return RangeImageUtils::computeRangeImage(intrinsics, xArray, yArray, zArray);
    }

    void setCloudPath(const std::string &path) {
        cloudPath = path;
    }

    std::string getCloudPath() {
        return cloudPath;
    }
}
