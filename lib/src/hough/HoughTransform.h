#pragma once
#include <optional>
#include <vector>

#include "hough/HoughStructs.h"
#include "point/PointArray.h"

namespace alice_lri {

    /**
     * @class HoughTransform
     * @brief A class to perform the Hough Transform for detecting lines in a 2D space.
     * Reference: https://en.wikipedia.org/wiki/Hough_transform
     */
    class HoughTransform {
    private:
        Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> accumulator;
        Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hashAccumulator;

        double xMin;
        double xMax;
        double xStep;
        double yMin;
        double yMax;
        double yStep;

        uint32_t xCount;
        uint32_t yCount;

    public:
        /**
         * @brief Constructor to initialize the HoughTransform with given parameters.
         * @param xMin Minimum x value.
         * @param xMax Maximum x value.
         * @param xStep Step size in x direction.
         * @param yMin Minimum y value.
         * @param yMax Maximum y value.
         * @param yStep Step size in y direction.
         */
        HoughTransform(double xMin, double xMax, double xStep, double yMin, double yMax, double yStep);

        /**
         * @brief Computes the accumulator array based on the given ranges and phis.
         * @param points
         */
        void computeAccumulator(const PointArray &points);

        /**
         * @brief Finds the maximum value in the accumulator and returns its coordinates.
         * @param averageX Optional average x value to find the closest maximum.
         * @return Optional pair of coordinates (x, y) of the maximum value.
         */
        std::optional<HoughCell> findMaximum(std::optional<double> averageX) const;

        /**
         * @brief Gets the x value given an index.
         * @param index The index.
         * @return The x value.
         */
        [[nodiscard]] double getXValue(size_t index) const;

        /**
         * @brief Gets the y value given an index.
         * @param index The index.
         * @return The y value.
         */
        [[nodiscard]] double getYValue(size_t index) const;

        [[nodiscard]] double getXMin() const {
            return xMin;
        }

        [[nodiscard]] double getXMax() const {
            return xMax;
        }

        [[nodiscard]] double getXStep() const {
            return xStep;
        }

        [[nodiscard]] double getYMin() const {
            return yMin;
        }

        [[nodiscard]] double getYMax() const {
            return yMax;
        }

        [[nodiscard]] double getYStep() const {
            return yStep;
        }

        [[nodiscard]] uint32_t getXCount() const {
            return xCount;
        }

        [[nodiscard]] uint32_t getYCount() const {
            return yCount;
        }

        void eraseByHash(uint64_t hash);

        void restoreVotes(uint64_t hash, int64_t votes);

        void addVotes(const PointArray &points, const Eigen::ArrayXi &indices);

        void removeVotes(const PointArray &points, const Eigen::ArrayXi &indices);

    private:
        /**
         * @brief Updates the accumulator for a specific point.
         * @param pointIndex Index of the point.
         * @param points Vector of phi values.
         * @param operation Multiplier for the vote value.
         * @param mode Mode to determine if hashes should be updated.
         */
        inline void updateAccumulatorForPoint(uint64_t pointIndex, const PointArray &points, HoughOperation operation, HoughMode mode);

        /**
         * @brief Votes for discontinuities in the accumulator to avoid gaps.
         *
         * This function ensures that there are no gaps in the accumulator array by voting not only for the current
         * y cell but also for the cells between the previous and current y values. As the accumulator is iterated
         * on the x-axis, if at any point the y value changes by more than 1, the function votes for the cells in
         * between the previous and current y values.
         * This is important because when detecting lines in a 2D space, line crossing might be missed if the
         * accumulator has gaps. By filling in these gaps, the function helps to create a more continuous and accurate
         * representation of the detected lines.
         *
         * @param pointIndex Index of the point being processed.
         * @param x The current x index in the accumulator.
         * @param y The current y value in the accumulator.
         * @param previousY The y value of the previous point.
         * @param operation
         * @param mode Mode to determine if hashes should be updated.
         */
        inline void voteForDiscontinuities(
            uint64_t pointIndex, int64_t x, int32_t y, int32_t previousY, HoughOperation operation, HoughMode mode
        );

        HoughCell indicesToCell(const std::pair<int64_t, int64_t> &indices) const;
    };
}
