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
    private:
        AliceArray<Scanline> scanlines;

    public:
        explicit Intrinsics(const int32_t scanlineCount): scanlines(scanlineCount) { }

        Scanline &scanlineAt(const int32_t idx) { return scanlines[idx]; }
        [[nodiscard]] const Scanline &scanlineAt(const int32_t idx) const { return scanlines[idx]; }
        [[nodiscard]] uint64_t scanlinesCount() const { return scanlines.size(); }
    };

    struct ACCURATE_RI_API RangeImage {
    private:
        AliceArray<double> pixels;
        uint32_t w;
        uint32_t h;

    public:
        RangeImage(): w(0), h(0) { }
        RangeImage(const uint32_t w, const uint32_t h): pixels(w * h), w(w), h(h) {};
        RangeImage(const uint32_t w, const uint32_t h, const double initialValue):
            pixels(w * h, initialValue), w(w), h(h) {};

        double& operator()(const uint32_t row, const uint32_t col) { return pixels[row * w + col]; }
        const double& operator()(const uint32_t row, const uint32_t col) const { return pixels[row * w + col]; }

        [[nodiscard]] uint32_t width() const { return w; }
        [[nodiscard]] uint32_t height() const { return h; }
        [[nodiscard]] const double* data() const { return pixels.data(); }
        double* data() { return pixels.data(); }
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

        [[nodiscard]] bool anyContained(const Interval &other) const;
        void clampBoth(double minValue, double maxValue);
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
