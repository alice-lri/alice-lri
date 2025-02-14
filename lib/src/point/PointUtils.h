#pragma once
#include <vector>

#include "PointArray.h"

namespace accurate_ri {
    class PointUtils {
    public:
        static Eigen::VectorXd computeRanges(const PointArray& points);
        static Eigen::VectorXd computePhis(const PointArray& points);
        static Eigen::VectorXd computeThetas(const PointArray& points);
        static double computeCoordsEps(const PointArray& points);
    };


} // accurate_ri
