#pragma once
#include <cstdint>
#include <Eigen/Core>

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

    template<typename T>
    typename T::Scalar medianInPlace(Eigen::ArrayBase<T> &array) {
        using Scalar = typename T::Scalar;
        const size_t n = array.size();
        size_t mid = n / 2;

        std::nth_element(array.begin(), array.begin() + mid, array.begin() + n);
        Scalar medianValue = array(mid);

        return medianValue;
    }
}
