#pragma once
#include <cstddef>
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API Array {
    private:
        struct Impl;
        Impl* impl = nullptr; // opaque

    public:
        Array();
        explicit Array(std::size_t n);

        Array(const Array &);
        Array &operator=(const Array &);

        Array(Array &&) noexcept;
        Array &operator=(Array &&) noexcept;

        ~Array() noexcept;

        std::size_t size() const noexcept;
        void resize(std::size_t n);
        T *data() noexcept;
        const T *data() const noexcept;

        T &operator[](std::size_t i) noexcept; // unchecked, like std::vector
        const T &operator[](std::size_t i) const noexcept;

        T &at(std::size_t i); // checked
        const T &at(std::size_t i) const;
    };

    extern template class ACCURATE_RI_API Array<float>;
    extern template class ACCURATE_RI_API Array<double>;
    extern template class ACCURATE_RI_API Array<Scanline>;
}
