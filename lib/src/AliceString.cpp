#include "accurate_ri/AliceString.h"
#include <string>

namespace accurate_ri {
    struct AliceString::Impl {
        std::string s;
    };

    AliceString::AliceString() noexcept : impl(new Impl) {}

    AliceString::AliceString(const char *c_str) noexcept : impl(new Impl) {
        if (c_str) {
            impl->s = c_str;
        }
    }

    AliceString::AliceString(const char *data, const uint64_t n) noexcept : impl(new Impl) {
        if (data && n > 0) {
            impl->s.assign(data, n);
        }
    }

    AliceString::AliceString(const AliceString &o) noexcept : impl(new Impl) {
        if (o.impl) {
            impl->s = o.impl->s;
        }
    }

    AliceString &AliceString::operator=(const AliceString &o) noexcept {
        if (this != &o) {
            AliceString tmp(o);
            std::swap(impl, tmp.impl);
        }
        return *this;
    }

    AliceString::AliceString(AliceString &&o) noexcept : impl(o.impl) {
        o.impl = new Impl;
    }

    AliceString &AliceString::operator=(AliceString &&o) noexcept {
        if (this != &o) {
            delete impl;
            impl = o.impl;
            o.impl = new Impl;
        }
        return *this;
    }

    AliceString &AliceString::operator=(const char *c_str) noexcept {
        if (c_str) {
            impl->s = c_str;
        }

        return *this;
    }

    AliceString::~AliceString() noexcept {
        delete impl;
    }

    uint64_t AliceString::size() const noexcept {
        return impl->s.size();
    }

    const char *AliceString::c_str() const noexcept {
        return impl->s.c_str();
    }

    bool AliceString::empty() const noexcept {
        return size() == 0;
    }

    char &AliceString::operator[](const uint64_t i) noexcept {
        return impl->s[i];
    }

    const char &AliceString::operator[](const uint64_t i) const noexcept {
        return impl->s[i];
    }

    void AliceString::clear() noexcept {
        impl->s.clear();
    }

    void AliceString::reserve(const uint64_t n) noexcept {
        impl->s.reserve(n);
    }

    void AliceString::shrink_to_fit() noexcept {
        impl->s.shrink_to_fit();
    }

    void AliceString::resize(const uint64_t n, const char fill) noexcept {
        impl->s.resize(n, fill);
    }

    void AliceString::push_back(const char ch) noexcept {
        impl->s.push_back(ch);
    }

    void AliceString::append(const char *c_str) noexcept {
        if (!c_str) {
            return;
        }

        impl->s.append(c_str);
    }

    void AliceString::append(const char *data, const uint64_t n) noexcept {
        if (!data || n == 0) {
            return;
        }

        impl->s.append(data, n);
    }

    void AliceString::append(const AliceString &other) noexcept {
        const char *buf = other.c_str();
        const uint64_t n = other.size();

        append(buf, n);
    }
} // accurate_ri
