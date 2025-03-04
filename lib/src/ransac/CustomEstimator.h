
#pragma once
#include <cstdint>

namespace accurate_ri {

class CustomEstimator {
private:
    const uint32_t resolution;

public:
    explicit CustomEstimator(const uint32_t resolution) : resolution(resolution) {}
};

} // accurate_ri
