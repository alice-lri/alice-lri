#include "accurate_ri.h"

#include "intrinsics/IntrinsicsEstimator.h"
#include "intrinsics/vertical/helper/VerticalLogging.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {
    // TODO remove this, the library should be path agnostic, just here for the trace file
    std::string cloudPath;

    void hello() {
        LOG_INFO("Hello");
        LOG_DEBUG("Hello2");
    }

    void execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
        Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(x.data(), x.size()).cast<double>();
        Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(y.data(), y.size()).cast<double>();
        Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(z.data(), z.size()).cast<double>();

        for (double& it : xArray) {
            it = std::rint(it * std::pow(10, 4)) / std::pow(10, 4);
        }

        for (double& it : yArray) {
            it = std::rint(it * std::pow(10, 4)) / std::pow(10, 4);
        }

        for (double& it : zArray) {
            it = std::rint(it * std::pow(10, 4)) / std::pow(10, 4);
        }

        const PointArray points(xArray, yArray, zArray);
        IntrinsicsEstimator estimator = IntrinsicsEstimator();

        estimator.estimate(points);
    }

    void setCloudPath(const std::string& path) {
        cloudPath = path;
    }

    std::string getCloudPath() {
        return cloudPath;
    }
}
