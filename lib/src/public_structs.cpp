#include <algorithm>
#include <vector>
#include <accurate_ri/public_structs.hpp>

namespace accurate_ri {
    struct RangeImage::Impl {
        std::vector<double> pixels;
    };

    RangeImage::RangeImage(const uint32_t w, const uint32_t h) : width(w), height(h), impl(new Impl()) {
        impl->pixels.resize(width * height);
    }

    RangeImage::RangeImage(
        const uint32_t w, const uint32_t h, const double val
    ) : width(w), height(h), impl(new Impl()) {
        impl->pixels.resize(width * height, val);
    }

    RangeImage::~RangeImage() {
        delete impl;
    }

    double &RangeImage::operator()(const uint32_t row, const uint32_t col) {
        return impl->pixels[row * width + col];
    }

    const double &RangeImage::operator()(const uint32_t row, const uint32_t col) const {
        return impl->pixels[row * width + col];
    }

    void RealMargin::clampBoth(const double minValue, const double maxValue) {
        lower = std::clamp(lower, minValue, maxValue);
        upper = std::clamp(upper, minValue, maxValue);
    }
}
