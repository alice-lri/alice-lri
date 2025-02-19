#include "accurate_ri.h"

#include "intrinsics/IntrinsicsEstimator.h"
#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri {
    void hello() {
        IntrinsicsEstimator estimator = IntrinsicsEstimator();
        LOG_INFO("Hello");
        LOG_DEBUG("Hello2");
    }

    void execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
        const Eigen::ArrayXd xArray = Eigen::Map<const Eigen::ArrayXf>(x.data(), x.size()).cast<double>();
        const Eigen::ArrayXd yArray = Eigen::Map<const Eigen::ArrayXf>(y.data(), y.size()).cast<double>();
        const Eigen::ArrayXd zArray = Eigen::Map<const Eigen::ArrayXf>(z.data(), z.size()).cast<double>();

        const PointArray points(std::move(xArray), std::move(yArray), std::move(zArray));
        IntrinsicsEstimator estimator = IntrinsicsEstimator();

        estimator.estimate(points);
    }
}
