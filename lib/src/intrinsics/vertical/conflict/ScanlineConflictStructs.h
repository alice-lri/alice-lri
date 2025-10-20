#pragma once
#include <unordered_set>
#include <Eigen/Core>

struct ScanlineConflicts {
    bool shouldReject;
    std::vector<uint32_t> conflictingScanlines;
};

struct HashToConflictValue {
    std::unordered_set<uint32_t> conflictingScanlines;
    int64_t votes;
};

struct ScanlineIntersectionFlags {
    std::vector<bool> empiricalIntersectionMask;
    std::vector<bool> theoreticalIntersectionMask;
    bool empiricalIntersection;
    bool theoreticalIntersection;

    [[nodiscard]] bool anyIntersection() const {
        return empiricalIntersection || theoreticalIntersection;
    }

    [[nodiscard]] bool anyIntersection(const uint32_t i) const {
        return empiricalIntersectionMask[i] || theoreticalIntersectionMask[i];
    }
};
struct ScanlineIntersectionInfo {
    ScanlineIntersectionFlags flags;
    std::vector<uint32_t> conflictingIds;
    std::vector<double> conflictingUncertainties;
};
