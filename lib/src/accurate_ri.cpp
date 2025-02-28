#include "accurate_ri.h"

#include "intrinsics/IntrinsicsEstimator.h"
#include "intrinsics/vertical/helper/JsonConverters.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {
    // TODO remove this, the library should be path agnostic, just here for the trace file
    std::string cloudPath;
    std::optional<std::string> outputPath = std::nullopt;

    void hello() {
        LOG_INFO("Hello");
        LOG_DEBUG("Hello2");
    }

    void execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(x.data(), x.size()).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(y.data(), y.size()).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(z.data(), z.size()).cast<double>();

        execute(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();
    }

    void execute(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXd>(x.data(), x.size());
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXd>(y.data(), y.size());
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXd>(z.data(), z.size());

        execute(xArray, yArray, zArray);
        PRINT_PROFILE_REPORT();
    }

    void execute(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray) {
        PROFILE_SCOPE("TOTAL");
        const PointArray points(xArray, yArray, zArray);
        IntrinsicsEstimator estimator = IntrinsicsEstimator();

        estimator.estimate(points);
    }

    void setCloudPath(const std::string &path) {
        cloudPath = path;
    }

    std::string getCloudPath() {
        return cloudPath;
    }

    void setOutputPath(const std::optional<std::string> &path) {
        outputPath = path;
    }

    std::optional<std::string> getOutputPath() {
        return outputPath;
    }
}
