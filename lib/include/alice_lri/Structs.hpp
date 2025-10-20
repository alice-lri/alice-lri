/**
 * @file Structs.hpp
 * @brief Data structures and type definitions for Alice LRI library.
 */
#pragma once
#include "alice_lri/ApiGuards.hpp"
#include "util/AliceArray.hpp"

namespace alice_lri {
    /**
     * @brief Represents a single scanline with intrinsic parameters.
     */
    struct ALICE_LRI_API Scanline {
        /** Vertical spatial offset of the scanline. */
        double verticalOffset;
        /** Vertical angle of the scanline. */
        double verticalAngle;
        /** Horizontal spatial offset of the scanline. */
        double horizontalOffset;
        /** Azimuthal offset of the scanline. */
        double azimuthalOffset;
        /** Horizontal resolution of the scanline. */
        int32_t resolution;
    };

    /**
     * @brief Contains intrinsic parameters for a sensor, including all scanlines.
     */
    struct ALICE_LRI_API Intrinsics {
        /** Array of scanlines describing the sensor geometry. */
        AliceArray<Scanline> scanlines;
        /**
         * @brief Constructs Intrinsics with a given number of scanlines.
         * @param scanlineCount Number of scanlines.
         */
        explicit Intrinsics(const int32_t scanlineCount) noexcept : scanlines(scanlineCount) { }
    };

    /**
     * @brief Represents a 2D range image with pixel data.
     */
    struct ALICE_LRI_API RangeImage {
    private:
        /** Pixel values (row-major order). */
        AliceArray<double> pixels;
        /** Image width. */
        uint32_t w;
        /** Image height. */
        uint32_t h;

    public:
        /** Default constructor (empty image). */
        RangeImage() noexcept : w(0), h(0) { }
    /**
     * @brief Construct with width and height. Reserves space for pixels but does not initialize them.
     * @param w Image width
     * @param h Image height
     */
    RangeImage(const uint32_t w, const uint32_t h) noexcept : pixels(), w(w), h(h) { pixels.reserve(w * h); }
        /**
         * @brief Construct with width, height, and initial pixel value.
         * @param w Image width
         * @param h Image height
         * @param initialValue Initial value for all pixels
         */
        RangeImage(const uint32_t w, const uint32_t h, const double initialValue) noexcept :
            pixels(w * h, initialValue), w(w), h(h) {}

        /**
         * @brief Access pixel at (row, col).
         */
        double& operator()(const uint32_t row, const uint32_t col) noexcept { return pixels[row * w + col]; }
        /**
         * @brief Access pixel at (row, col) (const).
         */
        const double& operator()(const uint32_t row, const uint32_t col) const noexcept { return pixels[row * w + col]; }

        /** @return Image width. */
        [[nodiscard]] uint32_t width() const noexcept { return w; }
        /** @return Image height. */
        [[nodiscard]] uint32_t height() const noexcept { return h; }
        /** @return Total number of pixels. */
        [[nodiscard]] uint64_t size() const noexcept { return w * h; }
        /** @return Pointer to pixel data (const). */
        [[nodiscard]] const double* data() const noexcept { return pixels.data(); }
        /** @return Pointer to pixel data. */
        double* data() noexcept { return pixels.data(); }
    };

    /**
     * @brief Namespace for point cloud data structures.
     */
    namespace PointCloud {
        /**
         * @brief 3D point cloud with float precision.
         */
        struct Float {
            /** X coordinates. */
            AliceArray<float> x;
            /** Y coordinates. */
            AliceArray<float> y;
            /** Z coordinates. */
            AliceArray<float> z;

            /**
             * @brief Reserve memory for a given number of points.
             * @param count Number of points to reserve.
             */
            void reserve(const uint64_t count) {
                x.reserve(count);
                y.reserve(count);
                z.reserve(count);
            }
        };

        /**
         * @brief 3D point cloud with double precision.
         */
        struct Double {
            /** X coordinates. */
            AliceArray<double> x;
            /** Y coordinates. */
            AliceArray<double> y;
            /** Z coordinates. */
            AliceArray<double> z;

            /**
             * @brief Reserve memory for a given number of points.
             * @param count Number of points to reserve.
             */
            void reserve(const uint64_t count) {
                x.reserve(count);
                y.reserve(count);
                z.reserve(count);
            }
        };
    }

    /**
     * @brief Represents a numeric interval [lower, upper].
     */
    struct ALICE_LRI_API Interval {
        /** Lower bound of the interval. */
        double lower;
        /** Upper bound of the interval. */
        double upper;

        /**
         * @brief Get the width of the interval.
         * @return Difference upper - lower.
         */
        [[nodiscard]] double diff() const noexcept {
            return upper - lower;
        }

        /**
         * @brief Check if any part of another interval is contained in this interval.
         * @param other The other interval.
         * @return True if any part is contained.
         */
        [[nodiscard]] bool anyContained(const Interval &other) const noexcept;

        /**
         * @brief Clamp both bounds to [minValue, maxValue].
         * @param minValue Minimum value.
         * @param maxValue Maximum value.
         */
        void clampBoth(double minValue, double maxValue) noexcept;
    };

    /**
     * @brief Value with associated confidence interval.
     */
    struct ALICE_LRI_API ValueConfInterval {
        /** The value. */
        double value;
        /** Confidence interval for the value. */
        Interval ci;
    };

    /**
     * @brief Angle bounds for a scanline.
     */
    struct ALICE_LRI_API ScanlineAngleBounds {
        /** Lower angle interval. */
        Interval lowerLine;
        /** Upper angle interval. */
        Interval upperLine;
    };

    /**
     * @brief Detailed scanline information with uncertainty and voting statistics.
     */
    struct ALICE_LRI_API ScanlineDetailed {
        /** Vertical spatial offset with confidence interval. */
        ValueConfInterval verticalOffset;
        /** Vertical angle with confidence interval. */
        ValueConfInterval verticalAngle;
        /** Horizontal spatial offset. */
        double horizontalOffset;
        /** Azimuthal offset. */
        double azimuthalOffset;
        /** Horizontal resolution of the scanline. */
        int32_t resolution;
        /** Estimated uncertainty. */
        double uncertainty;
        /** Number of Hough transform votes. */
        int64_t houghVotes;
        /** Hash value for Hough voting. */
        uint64_t houghHash;
        /** Number of points assigned to this scanline. */
        uint64_t pointsCount;
        /** Theoretical angle bounds for the scanline. */
        ScanlineAngleBounds theoreticalAngleBounds;
        /** Whether vertical heuristic was used. */
        bool verticalHeuristic;
        /** Whether horizontal heuristic was used. */
        bool horizontalHeuristic;
    };

    /**
     * @brief Reason for ending the iterative vertical fitting process.
     */
    enum class EndReason {
        ALL_ASSIGNED, /**< All points assigned. This is the normal termination condition. */
        MAX_ITERATIONS, /**< Maximum number of iterations reached. */
        NO_MORE_PEAKS /**< No more peaks found in the Hough accumulator. */
    };

    /**
     * @brief Detailed intrinsic parameters, including scanline details and statistics.
     */
    struct ALICE_LRI_API IntrinsicsDetailed {
        /** Array of detailed scanlines. */
        AliceArray<ScanlineDetailed> scanlines;
        /** Number of vertical iterations performed. */
        int32_t verticalIterations = 0;
        /** Number of unassigned points. */
        int32_t unassignedPoints = 0;
        /** Total number of points. */
        int32_t pointsCount = 0;
        /** Reason for ending the process. */
        EndReason endReason = EndReason::MAX_ITERATIONS;

        /**
         * @brief Construct with a given number of scanlines.
         * @param scanlineCount Number of scanlines.
         */
        explicit IntrinsicsDetailed(const int32_t scanlineCount) noexcept {
            scanlines.resize(scanlineCount);
        }

        /**
         * @brief Full constructor with all statistics.
         * @param scanlineCount Number of scanlines.
         * @param verticalIterations Number of vertical iterations.
         * @param unassignedPoints Number of unassigned points.
         * @param pointsCount Total number of points.
         * @param endReason Reason for ending the iterative algorithm.
         */
        IntrinsicsDetailed(
            const int32_t scanlineCount, const int32_t verticalIterations, const int32_t unassignedPoints,
            const int32_t pointsCount, const EndReason endReason
        ) noexcept : verticalIterations(verticalIterations), unassignedPoints(unassignedPoints),
            pointsCount(pointsCount),endReason(endReason) {
            scanlines.resize(scanlineCount);
        }
    };
}
