#pragma once
#include <cstdint>
#include <unordered_map>

#include "intrinsics/vertical/VerticalStructs.h"

namespace accurate_ri {
    class ScanlineConflictSolver {
    private:
        std::unordered_multimap<uint32_t, uint32_t> reverseScanlinesDependencyMap;
        std::unordered_map<uint64_t, HashToConflictValue> hashesToConflictsMap;
    };
} // accurate_ri
