#pragma once
#include <Eigen/Core>

namespace alice_lri::Utils {
    template<typename T>
    inline int8_t compare(const T &a, const T &b) {
        return (a < b) ? -1 : (a > b) ? 1 : 0;
    }

    template<typename T>
    inline int8_t sign(const T &a) {
        return compare(a, static_cast<T>(0));
    }

    template<typename T>
    inline auto diff(const Eigen::ArrayBase<T> &arr) {
        return arr.tail(arr.size() - 1) - arr.head(arr.size() - 1);
    }

    inline double positiveFmod(double x, double y) {
        return x - y * std::floor(x / y);
    }

    inline void positiveFmodInplace(Eigen::ArrayXd& x, double y) {
        x = x - y * (x / y).floor();
    }

    inline Eigen::ArrayXi eigenMaskToIndices(const Eigen::ArrayX<bool> &mask) {
        Eigen::ArrayXi indices(mask.count());

        int j = 0;
        for (int i = 0; i < mask.size(); ++i) {
            if (mask(i)) {
                indices(j++) = i;
            }
        }

        return indices;
    }
}
