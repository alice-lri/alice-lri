#pragma once
#include <cstddef>

#define ACCURATE_RI_API __attribute__((visibility("default")))

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API AliceArray {
    private:
        struct Impl;
        Impl* impl = nullptr; // opaque

    public:
        AliceArray();
        explicit AliceArray(std::size_t n);

        AliceArray(const AliceArray &);
        AliceArray &operator=(const AliceArray &);

        AliceArray(AliceArray &&) noexcept;
        AliceArray &operator=(AliceArray &&) noexcept;

        ~AliceArray() noexcept;

        std::size_t size() const noexcept;
        void resize(std::size_t n);
        T *data() noexcept;
        const T *data() const noexcept;

        T &operator[](std::size_t i) noexcept; // unchecked, like std::vector
        const T &operator[](std::size_t i) const noexcept;

        T &at(std::size_t i); // checked
        const T &at(std::size_t i) const;
    };
}
