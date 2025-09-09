#include <vector>
#include <algorithm>
#include <accurate_ri/public_structs.hpp>

namespace accurate_ri {

    void Interval::clampBoth(const double minValue, const double maxValue) {
        lower = std::clamp(lower, minValue, maxValue);
        upper = std::clamp(upper, minValue, maxValue);
    }

    struct Intrinsics::Impl {
      std::vector<Scanline> scanlines;
    };

    Intrinsics::Intrinsics(const int32_t scanlineCount): impl(new Impl()) {
        impl->scanlines.resize(scanlineCount);
    }

    Intrinsics::Intrinsics(const Intrinsics &other): impl(new Impl()) {
        impl->scanlines = other.impl->scanlines;
    }

    Intrinsics & Intrinsics::operator=(const Intrinsics &other) {
        if (this != &other) {
            Intrinsics tmp(other);
            std::swap(impl, tmp.impl);
        }

        return *this;
    }

    Intrinsics::Intrinsics(Intrinsics &&other) noexcept : impl(other.impl) {
        other.impl = nullptr;
    }

    Intrinsics& Intrinsics::operator=(Intrinsics&& other) noexcept {
        if (this != &other) {
            delete impl;
            impl = other.impl;
            other.impl = nullptr;
        }

        return *this;
    }

    Intrinsics::~Intrinsics() {
        delete impl;
    }

    Scanline& Intrinsics::scanlineAt(const int32_t idx) {
        if (!impl) {
            throw std::runtime_error("Intrinsics not initialized");
        }

        return impl->scanlines.at(idx);
    }

    const Scanline& Intrinsics::scanlineAt(const int32_t idx) const {
        if (!impl) {
            throw std::runtime_error("Intrinsics not initialized");
        }

        return impl->scanlines.at(idx);
    }

    int32_t Intrinsics::scanlinesCount() const {
        if (!impl) {
            throw std::runtime_error("Intrinsics object is not initialized (maybe moved-from?)");
        }

        return impl->scanlines.size();
    }

    struct RangeImage::Impl {
        std::vector<double> pixels;
        uint32_t width;
        uint32_t height;
    };

    RangeImage::RangeImage(const uint32_t width, const uint32_t height): impl(new Impl()) {
        const size_t size = width * height;
        impl->width = width;
        impl->height = height;
        impl->pixels.resize(size);
    }

    RangeImage::RangeImage(const uint32_t width, const uint32_t height, const double initialValue) : impl(new Impl()) {
        const size_t size = width * height;
        impl->width = width;
        impl->height = height;
        impl->pixels.resize(size, initialValue);
    }

    RangeImage::RangeImage(const RangeImage& other) : impl(new Impl()) {
        impl->width = other.impl->width;
        impl->height = other.impl->height;
        impl->pixels = other.impl->pixels;
    }

    RangeImage& RangeImage::operator=(const RangeImage& other) {
        if (this != &other) {
            RangeImage tmp(other);
            std::swap(impl, tmp.impl);
        }

        return *this;
    }

    RangeImage::RangeImage(RangeImage&& other) noexcept : impl(other.impl) {
        other.impl = nullptr;
    }

    RangeImage& RangeImage::operator=(RangeImage&& other) noexcept {
        if (this != &other) {
            delete impl;
            impl = other.impl;
            other.impl = nullptr;
        }

        return *this;
    }

    RangeImage::~RangeImage() {
        delete impl;
    }

    double& RangeImage::operator()(const uint32_t row, const uint32_t col) {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        const size_t index = row * impl->width + col;
        return impl->pixels.at(index);
    }

    const double& RangeImage::operator()(const uint32_t row, const uint32_t col) const {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        const size_t index = row * impl->width + col;
        return impl->pixels.at(index);
    }

    uint32_t RangeImage::width() const {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        return impl->width;
    }

    uint32_t RangeImage::height() const {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        return impl->height;
    }

    const double * RangeImage::data() const {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        return impl->pixels.data();
    }

    double * RangeImage::data() {
        if (!impl) {
            throw std::runtime_error("RangeImage object is not initialized (maybe moved-from?)");
        }

        return impl->pixels.data();
    }

    template class ACCURATE_RI_API AliceArray<DebugScanline>;
}
