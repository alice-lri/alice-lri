#pragma once
#include <cstdint>
#include "Api.h"

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API AliceArray {
    private:
        struct Impl;
        Impl* impl = nullptr;

    public:
        AliceArray() noexcept;
        explicit AliceArray(uint64_t n) noexcept;
        AliceArray(uint64_t n, const T &initialValue) noexcept;
        AliceArray(const T* data, uint64_t n) noexcept;

        AliceArray(const AliceArray &) noexcept;
        AliceArray &operator=(const AliceArray &) noexcept;
        AliceArray(AliceArray &&) noexcept;
        AliceArray &operator=(AliceArray &&) noexcept;
        ~AliceArray() noexcept;

        [[nodiscard]] uint64_t size() const noexcept;
        T *data() noexcept;
        const T *data() const noexcept;
        T &operator[](uint64_t i) noexcept;
        const T &operator[](uint64_t i) const noexcept;
        T *begin() noexcept { return data(); }
        T *end() noexcept { return data() + size(); }
        const T *begin() const noexcept { return data(); }
        const T *end() const noexcept { return data() + size(); }

        [[nodiscard]] bool empty() const noexcept;
        void push_back(const T &value) noexcept;
        void emplace_back(const T &value) noexcept;
        void resize(uint64_t n) noexcept;
        void reserve(uint64_t n) noexcept;
        void shrink_to_fit() noexcept;
        void clear() noexcept;
    };
}
