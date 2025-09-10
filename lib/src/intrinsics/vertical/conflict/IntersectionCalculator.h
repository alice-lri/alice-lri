#pragma once
#include "intrinsics/vertical/conflict/ScanlineConflictStructs.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"

namespace accurate_ri {
    class IntersectionCalculator {
    public:
        static ScanlineIntersectionFlags computeIntersectionFlags(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static ScanlineIntersectionInfo computeIntersectionInfo(
            const VerticalScanlinePool &scanlinePool, ScanlineIntersectionFlags &&flags
        );

    private:
        static std::vector<bool> computeEmpiricalIntersections(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );

        static std::vector<bool> computeTheoreticalIntersections(
            const VerticalScanlinePool &scanlinePool, const VerticalScanlineCandidate &candidate
        );
    };
} // accurate_ri