#include <vector>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <accurate_ri/public_structs.hpp>

namespace accurate_ri {

    struct RangeImage::Impl {
        std::vector<double> pixels;
    };

    RangeImage::RangeImage(const uint32_t width, const uint32_t height)
        : width(width), height(height), impl(new Impl()) {
        impl->pixels.resize(width * height);
    }

    RangeImage::RangeImage(const uint32_t width, const uint32_t height, const double initialValue)
        : width(width), height(height), impl(new Impl()) {
        impl->pixels.resize(width * height, initialValue);
    }

    RangeImage::RangeImage(const RangeImage& other)
        : width(other.width), height(other.height), impl(new Impl()) {
        impl->pixels = other.impl->pixels;
    }

    RangeImage& RangeImage::operator=(const RangeImage& other) {
        if (this != &other) {
            RangeImage tmp(other);
            std::swap(width, tmp.width);
            std::swap(height, tmp.height);
            std::swap(impl, tmp.impl);
        }

        return *this;
    }

    RangeImage::RangeImage(RangeImage&& other) noexcept
        : width(other.width), height(other.height), impl(other.impl) {
        other.impl = nullptr;
        other.width = 0;
        other.height = 0;
    }

    RangeImage& RangeImage::operator=(RangeImage&& other) noexcept {
        if (this != &other) {
            width = other.width;
            height = other.height;
            impl = other.impl;

            other.impl = nullptr;
            other.width = 0;
            other.height = 0;
        }

        return *this;
    }

    RangeImage::~RangeImage() {
        delete impl;
    }

    double& RangeImage::operator()(const uint32_t row, const uint32_t col) {
        assert(row < height && col < width);
        return impl->pixels[row * width + col];
    }

    const double& RangeImage::operator()(const uint32_t row, const uint32_t col) const {
        assert(row < height && col < width);
        return impl->pixels[row * width + col];
    }

}
