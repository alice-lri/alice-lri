#pragma once
#include <cstdint>

#define ACCURATE_RI_API __attribute__((visibility("default")))

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API AliceArray {
    private:
        struct Impl;
        Impl* impl = nullptr; // opaque

    public:
        AliceArray();
        explicit AliceArray(uint64_t n);

        AliceArray(const AliceArray &);
        AliceArray &operator=(const AliceArray &);

        AliceArray(AliceArray &&) noexcept;
        AliceArray &operator=(AliceArray &&) noexcept;

        ~AliceArray() noexcept;

        uint64_t size() const noexcept;
        void resize(uint64_t n);
        T *data() noexcept;
        const T *data() const noexcept;

        T &operator[](uint64_t i) noexcept; // unchecked, like std::vector
        const T &operator[](uint64_t i) const noexcept;

        T &at(uint64_t i); // checked
        const T &at(uint64_t i) const;
    };
}
