#pragma once
#include <string>
#include <vector>

namespace accurate_ri {
    void hello();
    void execute(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z);

    // TODO remove this, the library should be path agnostic, just here for the trace file
    void setCloudPath(const std::string& path);
    std::string getCloudPath();
}
