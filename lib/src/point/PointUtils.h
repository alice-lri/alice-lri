#pragma once
#include <vector>

#include "PointArray.h"

namespace accurate_ri {
    class PointUtils {
    public:
        static double computeCoordsEps(const PointArray& points);
    };


} // accurate_ri
