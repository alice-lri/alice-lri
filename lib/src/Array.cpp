#include "accurate_ri/Array.hpp"
#include <vector>
#include <cassert>

namespace accurate_ri {
    template<class T>
    struct Array<T>::Impl {
        std::vector<T> v;
    };

    template<class T>
    Array<T>::Array() : impl(new Impl) {}

    template<class T>
    Array<T>::Array(std::size_t n) : impl(new Impl) {
        impl->v.resize(n);
    }

    template<class T>
    Array<T>::Array(const Array &o) : impl(new Impl(*o.impl)) {}

    template<class T>
    Array<T> &Array<T>::operator=(const Array &o) {
        if (this != &o) {
            Array tmp(o);
            std::swap(impl, tmp.impl);
        }
        return *this;
    }

    template<class T>
    Array<T>::Array(Array &&o) noexcept : impl(o.impl) {
        o.impl = nullptr;
    }

    template<class T>
    Array<T> &Array<T>::operator=(Array &&o) noexcept {
        if (this != &o) {
            delete impl;
            impl = o.impl;
            o.impl = nullptr;
        }
        return *this;
    }

    template<class T>
    Array<T>::~Array() noexcept {
        delete impl;
    }

    template<class T>
    std::size_t Array<T>::size() const noexcept {
        return impl->v.size();
    }

    template<class T>
    void Array<T>::resize(std::size_t n) {
        impl->v.resize(n);
    }

    template<class T>
    T *Array<T>::data() noexcept {
        return impl->v.data();
    }

    template<class T>
    const T *Array<T>::data() const noexcept {
        return impl->v.data();
    }

    template<class T>
    T &Array<T>::operator[](std::size_t i) noexcept {
        return impl->v[i];
    }

    template<class T>
    const T &Array<T>::operator[](std::size_t i) const noexcept {
        return impl->v[i];
    }

    template<class T>
    T &Array<T>::at(std::size_t i) {
        return impl->v.at(i);
    }

    template<class T>
    const T &Array<T>::at(std::size_t i) const {
        return impl->v.at(i);
    }
}
