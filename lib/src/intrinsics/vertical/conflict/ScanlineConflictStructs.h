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

struct ScanlineIntersectionFlags {
    Eigen::ArrayX<bool> empiricalIntersectionMask;
    Eigen::ArrayX<bool> theoreticalIntersectionMask;
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
    Eigen::ArrayXi conflictingIds;
    Eigen::ArrayXd conflictingUncertainties;
};
