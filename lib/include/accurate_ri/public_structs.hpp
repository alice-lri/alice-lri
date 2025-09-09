#pragma once
#include <cstdint>
#include <vector>

#define ACCURATE_RI_API __attribute__((visibility("default")))

namespace accurate_ri {
    struct ACCURATE_RI_API Scanline {
        double verticalOffset;
        double verticalAngle;
        double horizontalOffset;
        double azimuthalOffset;
        int32_t resolution;
    };

    struct ACCURATE_RI_API Intrinsics {
    private:
        struct Impl;
        Impl* impl;

    public:
        explicit Intrinsics(int32_t scanlineCount);
        Intrinsics(const Intrinsics& other);
        Intrinsics& operator=(const Intrinsics& other);
        Intrinsics(Intrinsics&& other) noexcept;
        Intrinsics& operator=(Intrinsics&& other) noexcept;

        ~Intrinsics();

        Scanline &scanlineAt(int32_t idx); // like vector::at
        const Scanline &scanlineAt(int32_t idx) const;
        int32_t scanlinesCount() const;
    };

    struct ACCURATE_RI_API RangeImage {
    private:
        struct Impl;
        Impl* impl;

    public:
        RangeImage(uint32_t width, uint32_t height);
        RangeImage(uint32_t width, uint32_t height, double initialValue);

        RangeImage(const RangeImage& other);
        RangeImage& operator=(const RangeImage& other);

        RangeImage(RangeImage&& other) noexcept;
        RangeImage& operator=(RangeImage&& other) noexcept;

        ~RangeImage();

        double& operator()(uint32_t row, uint32_t col);
        const double& operator()(uint32_t row, uint32_t col) const;

        uint32_t width() const;
        uint32_t height() const;
        const double* data() const;
        double* data();
    };

    namespace PointCloud {
        struct ACCURATE_RI_API Float {
            std::vector<float> x;
            std::vector<float> y;
            std::vector<float> z;
        };

        struct ACCURATE_RI_API Double {
            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> z;
        };
    }
}
