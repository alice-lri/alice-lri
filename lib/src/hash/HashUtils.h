#pragma once
#include <cstdint>

namespace alice_lri {

class HashUtils {
public:
    static uint64_t knuthHash(uint64_t x);
};

}
