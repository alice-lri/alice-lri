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

    /**
     * \brief Computes the median of the input array in-place.
     *
     * This function rearranges the elements of the input array such that the median
     * value is placed in its correct position. The input array is modified during
     * the computation, and its original order is not preserved.
     *
     * \tparam T The type of the Eigen array.
     * \param array The input Eigen array, which will be modified in-place.
     * \return The median value of the input array.
     *
     * \note This function invalidates the input array by modifying its order.
     */
    template<typename T>
    typename T::Scalar medianInPlace(Eigen::ArrayBase<T> &array) {
        using Scalar = typename T::Scalar;
        const size_t n = array.size();
        size_t mid = n / 2;

        std::nth_element(array.begin(), array.begin() + mid, array.begin() + n);
        Scalar medianValue = array(mid);

        return medianValue;
    }

    // TODO make sure to use this function everywhere
    inline auto eigenMaskToIndices(const Eigen::ArrayX<bool> &mask) {
        std::vector<int32_t> indicesVector;
        indicesVector.reserve(mask.size());

        for (int i = 0; i < mask.size(); ++i) {
            if (mask(i)) {
                indicesVector.emplace_back(i);
            }
        }

        return Eigen::Map<const Eigen::ArrayXi>(indicesVector.data(), indicesVector.size());
    }
}
