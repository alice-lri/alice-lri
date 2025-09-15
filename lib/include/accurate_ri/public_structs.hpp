#pragma once
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
        AliceArray<Scanline> scanlines;
        explicit Intrinsics(const int32_t scanlineCount) noexcept : scanlines(scanlineCount) { }
    };

    struct ACCURATE_RI_API RangeImage {
    private:
        AliceArray<double> pixels;
        uint32_t w;
        uint32_t h;

    public:
        RangeImage() noexcept : w(0), h(0) { }
        RangeImage(const uint32_t w, const uint32_t h) noexcept : pixels(w * h), w(w), h(h) {}
        RangeImage(const uint32_t w, const uint32_t h, const double initialValue) noexcept :
            pixels(w * h, initialValue), w(w), h(h) {}

        double& operator()(const uint32_t row, const uint32_t col) noexcept { return pixels[row * w + col]; }
        const double& operator()(const uint32_t row, const uint32_t col) const noexcept { return pixels[row * w + col]; }

        [[nodiscard]] uint32_t width() const noexcept { return w; }
        [[nodiscard]] uint32_t height() const noexcept { return h; }
        [[nodiscard]] uint64_t size() const noexcept { return w * h; }
        [[nodiscard]] const double* data() const noexcept { return pixels.data(); }
        double* data() noexcept { return pixels.data(); }
    };

    namespace PointCloud {
        struct Float {
            AliceArray<float> x;
            AliceArray<float> y;
            AliceArray<float> z;

            void reserve(const uint64_t count) {
                x.reserve(count);
                y.reserve(count);
                z.reserve(count);
            }
        };

        struct Double {
            AliceArray<double> x;
            AliceArray<double> y;
            AliceArray<double> z;

            void reserve(const uint64_t count) {
                x.reserve(count);
                y.reserve(count);
                z.reserve(count);
            }
        };
    }

    struct ACCURATE_RI_API Interval {
        double lower;
        double upper;

        [[nodiscard]] double diff() const noexcept {
            return upper - lower;
        }

        [[nodiscard]] bool anyContained(const Interval &other) const noexcept;
        void clampBoth(double minValue, double maxValue) noexcept;
    };

    struct ACCURATE_RI_API ValueConfInterval {
        double value;
        Interval ci;
    };

    struct ACCURATE_RI_API ScanlineAngleBounds {
        Interval lowerLine;
        Interval upperLine;
    };

    struct ACCURATE_RI_API DebugScanline {
        ValueConfInterval verticalOffset;
        ValueConfInterval verticalAngle;
        double horizontalOffset;
        double azimuthalOffset;
        int32_t resolution;
        double uncertainty;
        int64_t houghVotes;
        uint64_t houghHash;
        uint64_t pointsCount;
        ScanlineAngleBounds theoreticalAngleBounds;
        bool verticalHeuristic;
        bool horizontalHeuristic;
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

        explicit DebugIntrinsics(const int32_t scanlineCount) noexcept {
            scanlines.resize(scanlineCount);
        }

        DebugIntrinsics(
            const int32_t scanlineCount, const int32_t verticalIterations, const int32_t unassignedPoints,
            const int32_t pointsCount, const EndReason endReason
        ) noexcept : verticalIterations(verticalIterations), unassignedPoints(unassignedPoints),
            pointsCount(pointsCount),endReason(endReason) {
            scanlines.resize(scanlineCount);
        }
    };
}
