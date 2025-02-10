#include "HashUtils.h"

#define MULTIPLIER 11400714819323198485ULL // Knuth's 64-bit multiplier
#define OFFSET 1

namespace accurate_ri {
uint64_t HashUtils::knuth_uint(uint64_t x) {
    uint64_t hash_value = (x + OFFSET) * MULTIPLIER;
    return hash_value;
}
} // accurate_ri