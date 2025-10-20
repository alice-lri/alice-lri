#include "HashUtils.h"

constexpr uint64_t MULTIPLIER = 11400714819323198485ULL; // Knuth's 64-bit multiplier
constexpr uint64_t OFFSET = 1;

namespace alice_lri {
    uint64_t HashUtils::knuthHash(const uint64_t x) {
        return (x + OFFSET) * MULTIPLIER;
    }
}
