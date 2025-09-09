#pragma once
#include <unordered_set>
#include <Eigen/Core>

struct ScanlineConflictsResult {
    bool shouldReject;
    Eigen::ArrayXi conflictingScanlines;
};

struct HashToConflictValue {
    std::unordered_set<uint32_t> conflictingScanlines;
    int64_t votes;
};

struct ScanlineIntersectionInfo {
    const Eigen::ArrayX<bool> empiricalIntersectionMask;
    const Eigen::ArrayX<bool> theoreticalIntersectionMask;
    const bool empiricalIntersection;
    const bool theoreticalIntersection;

    [[nodiscard]] bool anyIntersection() const {
        return empiricalIntersection || theoreticalIntersection;
    }

    [[nodiscard]] bool anyIntersection(const uint32_t i) const {
        return empiricalIntersectionMask[i] || theoreticalIntersectionMask[i];
    }
};
