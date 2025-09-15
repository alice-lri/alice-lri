#include "alice_lri/AliceArray.hpp"
#include "alice_lri/public_structs.hpp"
#include <vector>
#include <utility>

namespace alice_lri {
    template<class T>
    struct AliceArray<T>::Impl {
        std::vector<T> v;
    };

    template<class T>
    AliceArray<T>::AliceArray() noexcept : impl(new Impl) {}

    template<class T>
    AliceArray<T>::AliceArray(uint64_t n) noexcept : impl(new Impl) {
        impl->v.resize(n);
    }

    template<class T>
    AliceArray<T>::AliceArray(uint64_t n, const T &initialValue) noexcept : impl(new Impl) {
        impl->v.resize(n, initialValue);
    }

    template<class T>
    AliceArray<T>::AliceArray(const T* data, uint64_t n) noexcept : impl(new Impl) {
        impl->v.assign(data, data + n);
    }

    template<class T>
    AliceArray<T>::AliceArray(const AliceArray &o) noexcept: impl(new Impl(*o.impl)) {}

    template<class T>
    AliceArray<T> &AliceArray<T>::operator=(const AliceArray &o) noexcept {
        if (this != &o) {
            AliceArray tmp(o);
            std::swap(impl, tmp.impl);
        }
        return *this;
    }

    template<class T>
    AliceArray<T>::AliceArray(AliceArray &&o) noexcept : impl(o.impl) {
        o.impl = new Impl();
    }

    template<class T>
    AliceArray<T> &AliceArray<T>::operator=(AliceArray &&o) noexcept {
        if (this != &o) {
            delete impl;
            impl = o.impl;
            o.impl = new Impl();
        }
        return *this;
    }

    template<class T>
    AliceArray<T>::~AliceArray() noexcept {
        delete impl;
    }

    template<class T>
    uint64_t AliceArray<T>::size() const noexcept {
        return impl->v.size();
    }

    template<class T>
    void AliceArray<T>::resize(uint64_t n) noexcept {
        impl->v.resize(n);
    }

    template<class T>
    bool AliceArray<T>::empty() const noexcept {
        return impl->v.empty();
    }

    template<class T>
    void AliceArray<T>::push_back(const T &value) noexcept {
        impl->v.push_back(value);
    }

    template<class T>
    void AliceArray<T>::emplace_back(const T &value) noexcept {
        impl->v.emplace_back(value);
    }

    template<class T>
    void AliceArray<T>::reserve(uint64_t n) noexcept {
        impl->v.reserve(n);
    }

    template<class T>
    void AliceArray<T>::shrink_to_fit() noexcept {
        impl->v.shrink_to_fit();
    }

    template<class T>
    void AliceArray<T>::clear() noexcept {
        impl->v.clear();
    }

    template<class T>
    T *AliceArray<T>::data() noexcept {
        return impl->v.data();
    }

    template<class T>
    const T *AliceArray<T>::data() const noexcept {
        return impl->v.data();
    }

    template<class T>
    T &AliceArray<T>::operator[](uint64_t i) noexcept {
        return impl->v[i];
    }

    template<class T>
    const T &AliceArray<T>::operator[](uint64_t i) const noexcept {
        return impl->v[i];
    }

    template class AliceArray<float>;
    template class AliceArray<double>;
    template class AliceArray<Scanline>;
    template class AliceArray<DebugScanline>;
}
