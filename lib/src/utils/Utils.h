#pragma once
#include <cstdint>
#include <eigen3/Eigen/Core>

namespace accurate_ri::Utils {
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
}
