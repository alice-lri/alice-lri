#pragma once
#include <vector>

#include "PointArray.h"

namespace accurate_ri {
    class PointUtils {
    public:
        static std::vector<double> computeRanges(const PointArray& points);
        static std::vector<double> computePhis(const PointArray& points);
        static std::vector<double> computeThetas(const PointArray& points);
        static double computeCoordsEps(const PointArray& points);
    };


} // accurate_ri
