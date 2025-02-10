#pragma once
#include <vector>

namespace accurate_ri {

class IntrinsicsEstimator {

public:
    void estimate(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z);
};

} // accurate_ri
