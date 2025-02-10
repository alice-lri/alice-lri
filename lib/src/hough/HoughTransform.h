#pragma once
#include <cstdint>
#include <vector>

namespace accurate_ri {
    /**
     * @class HoughTransform
     * @brief A class to perform the Hough Transform for detecting lines in a 2D space.
     * Reference: https://en.wikipedia.org/wiki/Hough_transform
     */
    class HoughTransform {
    private:
        std::vector<double> accumulator; ///< Accumulator array for votes.
        std::vector<uint64_t> hashAccumulator; ///< Hash accumulator for unique point combination identification.

        double xMin; ///< Minimum x value.
        double xMax; ///< Maximum x value.
        double xStep; ///< Step size in x direction.
        double yMin; ///< Minimum y value.
        double yMax; ///< Maximum y value.
        double yStep; ///< Step size in y direction.

        uint32_t xCount; ///< Number of steps in x direction.
        uint32_t yCount; ///< Number of steps in y direction.

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
         * @param ranges Vector of range values.
         * @param phis Vector of phi values.
         */
        void computeAccumulator(const std::vector<double> &ranges, const std::vector<double> &phis);

    private:
        /**
         * @brief Updates the accumulator for a specific point.
         * @param pointIndex Index of the point.
         * @param ranges Vector of range values.
         * @param phis Vector of phi values.
         * @param xValues Precomputed x values.
         */
        inline void updateAccumulatorForPoint(uint64_t pointIndex, const std::vector<double> &ranges,
                                              const std::vector<double> &phis,
                                              const std::vector<double> &xValues);

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
         * @param voteVal The value to be added to the accumulator for the current point.
         * @param previousY The y value of the previous point.
         */
        inline void voteForDiscontinuities(uint64_t pointIndex, size_t x, int32_t y, double voteVal, int32_t previousY);
    };
} // accurate_ri
