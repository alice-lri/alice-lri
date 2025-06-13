#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>

namespace accurate_ri {
    struct RealMargin {
        double lower;
        double upper;

        [[nodiscard]] double diff() const {
            return upper - lower;
        }

        inline void clampBoth(const double minValue, const double maxValue) {
            lower = std::clamp(lower, minValue, maxValue);
            upper = std::clamp(upper, minValue, maxValue);
        }
    };

    struct ScanlineAngleBounds {
        RealMargin bottom;
        RealMargin top;
    };

    struct OffsetAngleMargin {
        RealMargin offset;
        RealMargin angle;
    };

    struct OffsetAngle {
        double offset;
        double angle;
    };

    struct ScanlineInfo {
        uint32_t id;
        uint64_t pointsCount;
        OffsetAngle values;
        OffsetAngleMargin ci;
        ScanlineAngleBounds theoreticalAngleBounds;
        std::vector<uint32_t> dependencies;
        double uncertainty;
        double houghVotes;
        uint64_t houghHash;
    };

    struct FullScanlines {
        std::vector<ScanlineInfo> scanlines;
        std::vector<int> pointsScanlinesIds;
    };

    enum class EndReason {
        ALL_ASSIGNED, MAX_ITERATIONS, NO_MORE_PEAKS
    };

    struct ScanlineHorizontalInfo { // TODO maybe rename to HorizontalScanlineInfo
        int32_t resolution;
        double offset;
        double thetaOffset;
        bool heuristic;
    };

    struct HorizontalIntrinsicsResult {
        std::vector<ScanlineHorizontalInfo> scanlines;
    };

    struct VerticalIntrinsicsResult {
        uint32_t iterations = 0;
        uint32_t scanlinesCount = 0;
        uint32_t unassignedPoints = 0;
        uint32_t pointsCount = 0;
        EndReason endReason = EndReason::MAX_ITERATIONS;
        FullScanlines fullScanlines;
    };

    struct IntrinsicsResult {
        VerticalIntrinsicsResult vertical;
        HorizontalIntrinsicsResult horizontal;
    };

    struct RangeImage {
        const uint32_t width;
        const uint32_t height;

    private:
        std::vector<double> pixels;

    public:
        RangeImage(const uint32_t width, const uint32_t height) : width(width), height(height) {
            pixels.resize(width * height);
        }

        RangeImage(const uint32_t width, const uint32_t height, const double initialValue)
            : width(width), height(height) {
            pixels.resize(width * height, initialValue);
        }

        double &operator()(const uint32_t row, const uint32_t col) {
            return pixels[row * width + col];
        }

        const double &operator()(const uint32_t row, const uint32_t col) const {
            return pixels[row * width + col];
        }
    };

    namespace PointCloud {
        struct Float {
            std::vector<float> x;
            std::vector<float> y;
            std::vector<float> z;
        };

        struct Double {
            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> z;
        };
    }
}
