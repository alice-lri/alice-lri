#pragma once
#include <cstdint>
#include <vector>

#define ACCURATE_RI_API __attribute__((visibility("default")))


namespace accurate_ri {
    struct ACCURATE_RI_API RealMargin {
        double lower;
        double upper;

        [[nodiscard]] double diff() const {
            return upper - lower;
        }

        void clampBoth(double minValue, double maxValue);
    };

    struct ACCURATE_RI_API ScanlineAngleBounds {
        RealMargin bottom;
        RealMargin top;
    };

    struct ACCURATE_RI_API OffsetAngleMargin {
        RealMargin offset;
        RealMargin angle;
    };

    struct ACCURATE_RI_API OffsetAngle {
        double offset;
        double angle;
    };

    struct ACCURATE_RI_API ScanlineInfo {
        uint32_t id;
        uint64_t pointsCount;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineAngleBounds theoreticalAngleBounds;
        double uncertainty;
        double houghVotes;
        uint64_t houghHash;
    };

    struct ACCURATE_RI_API FullScanlines {
        std::vector<ScanlineInfo> scanlines;
        std::vector<int> pointsScanlinesIds;
    };

    enum class ACCURATE_RI_API EndReason {
        ALL_ASSIGNED, MAX_ITERATIONS, NO_MORE_PEAKS
    };

    struct ACCURATE_RI_API ScanlineHorizontalInfo { // TODO maybe rename to HorizontalScanlineInfo
        int32_t resolution;
        double offset;
        double thetaOffset;
        bool heuristic;
    };

    struct ACCURATE_RI_API HorizontalIntrinsicsResult {
        std::vector<ScanlineHorizontalInfo> scanlines;
    };

    struct ACCURATE_RI_API VerticalIntrinsicsResult {
        uint32_t iterations = 0;
        uint32_t scanlinesCount = 0;
        uint32_t unassignedPoints = 0;
        uint32_t pointsCount = 0;
        EndReason endReason = EndReason::MAX_ITERATIONS;
        FullScanlines fullScanlines;
    };

    struct ACCURATE_RI_API IntrinsicsResult { // TODO refactor this (should be per scanline)
        VerticalIntrinsicsResult vertical;
        HorizontalIntrinsicsResult horizontal;
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
