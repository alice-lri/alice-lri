#include "accurate_ri/Utils.hpp"
#include "accurate_ri/public_structs.hpp"
#include <vector>
#include <cassert>

namespace accurate_ri {
    template<class T>
    struct AliceArray<T>::Impl {
        std::vector<T> v;
    };

    template<class T>
    AliceArray<T>::AliceArray() : impl(new Impl) {}

    template<class T>
    AliceArray<T>::AliceArray(uint64_t n) : impl(new Impl) {
        impl->v.resize(n);
    }

    template<class T>
    AliceArray<T>::AliceArray(const AliceArray &o) : impl(new Impl(*o.impl)) {}

    template<class T>
    AliceArray<T> &AliceArray<T>::operator=(const AliceArray &o) {
        if (this != &o) {
            AliceArray tmp(o);
            std::swap(impl, tmp.impl);
        }
        return *this;
    }

    template<class T>
    AliceArray<T>::AliceArray(AliceArray &&o) noexcept : impl(o.impl) {
        o.impl = nullptr;
    }

    template<class T>
    AliceArray<T> &AliceArray<T>::operator=(AliceArray &&o) noexcept {
        if (this != &o) {
            delete impl;
            impl = o.impl;
            o.impl = nullptr;
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
    void AliceArray<T>::resize(uint64_t n) {
        impl->v.resize(n);
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

    template<class T>
    T &AliceArray<T>::at(uint64_t i) {
        return impl->v.at(i);
    }

    template<class T>
    const T &AliceArray<T>::at(uint64_t i) const {
        return impl->v.at(i);
    }

    template class AliceArray<float>;
    template class AliceArray<double>;
    template class AliceArray<Scanline>;
    template class AliceArray<DebugScanline>;
}
