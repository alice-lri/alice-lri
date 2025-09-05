#pragma once
#include <cstdint>

namespace accurate_ri {

class HashUtils {
public:
    static uint64_t knuthHash(uint64_t x);
};

} // accurate_ri
