#pragma once
#include <vector>

namespace accurate_ri {
    class PointUtils {
    public:
        static std::vector<double> computeRanges(const std::vector<double> &x,
                                                 const std::vector<double> &y,
                                                 const std::vector<double> &z);

        static std::vector<double> computePhis(const std::vector<double> &x,
                                               const std::vector<double> &z,
                                               const std::vector<double> &ranges);

        static std::vector<double> computeThetas(const std::vector<double> &x,
                                                 const std::vector<double> &y);
    };
} // accurate_ri
