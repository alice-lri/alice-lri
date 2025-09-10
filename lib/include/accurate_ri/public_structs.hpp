#pragma once
#include <cstdint>
#include <vector>

#include "accurate_ri/AliceArray.hpp"

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

        Scanline &scanlineAt(int32_t idx);
        [[nodiscard]] const Scanline &scanlineAt(int32_t idx) const;
        [[nodiscard]] int32_t scanlinesCount() const;
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

        [[nodiscard]] uint32_t width() const;
        [[nodiscard]] uint32_t height() const;
        [[nodiscard]] const double* data() const;
        double* data();
    };

    namespace PointCloud {
        struct Float {
            AliceArray<float> x;
            AliceArray<float> y;
            AliceArray<float> z;
        };

        struct Double {
            AliceArray<double> x;
            AliceArray<double> y;
            AliceArray<double> z;
        };
    }

    struct ACCURATE_RI_API Interval {
        double lower;
        double upper;

        [[nodiscard]] double diff() const {
            return upper - lower;
        }

        void clampBoth(double minValue, double maxValue);
    };

    struct ACCURATE_RI_API ValueConfInterval {
        double value;
        Interval ci;
    };

    struct ACCURATE_RI_API ScanlineAngleBounds {
        Interval bottom;
        Interval top;
    };

    struct ACCURATE_RI_API DebugScanline {
        ValueConfInterval verticalOffset;
        ValueConfInterval verticalAngle;
        double horizontalOffset;
        double azimuthalOffset;
        int32_t resolution;
        bool heuristic;
        double uncertainty;
        int64_t houghVotes;
        uint64_t houghHash;
        uint64_t pointsCount;
        ScanlineAngleBounds theoreticalAngleBounds;
    };

    enum class ACCURATE_RI_API EndReason {
        ALL_ASSIGNED, MAX_ITERATIONS, NO_MORE_PEAKS
    };

    struct ACCURATE_RI_API DebugIntrinsics {
        AliceArray<DebugScanline> scanlines;
        int32_t verticalIterations = 0;
        int32_t unassignedPoints = 0;
        int32_t pointsCount = 0;
        EndReason endReason = EndReason::MAX_ITERATIONS;

        explicit DebugIntrinsics(const int32_t scanlineCount) {
            scanlines.resize(scanlineCount);
        }

        DebugIntrinsics(
            const int32_t scanlineCount, const int32_t verticalIterations, const int32_t unassignedPoints,
            const int32_t pointsCount, const EndReason endReason
        ) : verticalIterations(verticalIterations), unassignedPoints(unassignedPoints), pointsCount(pointsCount),
            endReason(endReason) {
            scanlines.resize(scanlineCount);
        }
    };
}
