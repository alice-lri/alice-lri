#pragma once
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    void hello();

    IntrinsicsResult execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z);

    IntrinsicsResult execute(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z);

    IntrinsicsResult execute(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray);

    void writeToJson(const IntrinsicsResult &result, const std::string &outputPath);

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const std::vector<float> &x, const std::vector<float> &y,
        const std::vector<float> &z
    );

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const std::vector<double> &x, const std::vector<double> &y,
        const std::vector<double> &z
    );

    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray,
        const Eigen::ArrayXd &zArray
    );

    // TODO remove this, the library should be path agnostic, just here for the trace file
    void setCloudPath(const std::string &path);

    std::string getCloudPath();
}
