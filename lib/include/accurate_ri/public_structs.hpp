#pragma once
#include <cstdint>
#include <vector>

#define ACCURATE_RI_API __attribute__((visibility("default")))


namespace accurate_ri {

    struct ACCURATE_RI_API ScanlineHorizontalInfo { // TODO maybe rename to HorizontalScanlineInfo
        int32_t resolution;
        double offset;
        double thetaOffset;
        bool heuristic;
    };

    struct ACCURATE_RI_API HorizontalIntrinsicsResult {
        std::vector<ScanlineHorizontalInfo> scanlines;
    };

    struct ACCURATE_RI_API Intrinsics { // TODO refactor this (should be per scanline)

    };

    struct ACCURATE_RI_API RangeImage {
        uint32_t width;
        uint32_t height;

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
