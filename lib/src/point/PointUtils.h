#pragma once
#include <vector>

#include "PointArray.h"

namespace accurate_ri {
    class PointUtils {
    public:
        template <PointArrayLayout T>
        static std::vector<double> computeRanges(const PointArray<T>& points);

        template <PointArrayLayout T>
        static std::vector<double> computePhis(const PointArray<T>& points);

        template <PointArrayLayout T>
        static std::vector<double> computeThetas(const PointArray<T>& points);

        template <PointArrayLayout T>
        static double computeCoordsEps(const PointArray<T>& points);
    };


} // accurate_ri
