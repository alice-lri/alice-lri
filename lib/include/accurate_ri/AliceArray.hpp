#pragma once
#include <cstdint>

#if defined(_MSC_VER)
  #define ACCURATE_RI_API __declspec(dllexport /* or dllimport */)
#else
  #define ACCURATE_RI_API __attribute__((visibility("default")))
#endif

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API AliceArray {
    private:
        struct Impl;
        Impl* impl = nullptr;

    public:
        AliceArray();
        explicit AliceArray(uint64_t n);
        AliceArray(uint64_t n, const T &initialValue);
        AliceArray(const T* data, uint64_t n);

        AliceArray(const AliceArray &);
        AliceArray &operator=(const AliceArray &);
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

        void push_back(const T &value);
        void emplace_back(const T &value);
        void resize(uint64_t n);
        void reserve(uint64_t n);
        void shrink_to_fit();
        void clear();
    };
}
