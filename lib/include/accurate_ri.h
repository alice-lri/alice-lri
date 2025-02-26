#pragma once
#include <optional>
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

namespace accurate_ri {
    void hello();
    void execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z);
    void execute(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z);
    void execute(const Eigen::ArrayXd &xArray, const Eigen::ArrayXd &yArray, const Eigen::ArrayXd &zArray);

    // TODO remove this, the library should be path agnostic, just here for the trace file
    void setCloudPath(const std::string& path);
    std::string getCloudPath();

    void setOutputPath(const std::optional<std::string>& path);
    std::optional<std::string> getOutputPath();
}
