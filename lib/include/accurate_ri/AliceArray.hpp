#pragma once
#include <cstdint>

#if defined(_MSC_VER)
  #define ACCURATE_RI_API __declspec(dllexport /* or dllimport */)
#else
  #define ACCURATE_RI_API __attribute__((visibility("default")))
#endif

namespace accurate_ri {
    template<class T>
    class ACCURATE_RI_API AliceSpan {
    private:
        T* _data = nullptr;
        uint64_t _size = 0;

    public:
        constexpr AliceSpan() noexcept = default;
        AliceSpan(T *data, const uint64_t n) : _data(data), _size(n) {}

        [[nodiscard]] uint64_t size() const noexcept { return _size; }
        T *data() noexcept { return _data; }
        const T *data() const noexcept { return _data; }
        T &operator[](std::size_t i) noexcept { return _data[i]; }
        const T &operator[](std::size_t i) const noexcept { return _data[i]; }
        T *begin() noexcept { return _data; }
        T *end() noexcept { return _data + _size; }
        const T *begin() const noexcept { return _data; }
        const T *end() const noexcept { return _data + _size; }
    };

    template<class T>
    class ACCURATE_RI_API AliceArray {
    private:
        struct Impl;
        Impl* impl = nullptr;

    public:
        AliceArray();
        explicit AliceArray(uint64_t n);
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

        AliceSpan<T> span() noexcept { return {data(), size()}; }
        AliceSpan<const T> span() const noexcept { return {data(), size()}; }
    };
}
