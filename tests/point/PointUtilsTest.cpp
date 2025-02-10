#include <gtest/gtest.h>
#include "point/PointUtils.h"
#include <vector>
#include <cmath>

TEST(PointUtilsTest, ComputeRanges) {
    std::vector<double> x = {1.0, 2.0, 3.0};
    std::vector<double> y = {4.0, 5.0, 6.0};
    std::vector<double> z = {7.0, 8.0, 9.0};
    std::vector<double> expected_ranges = {std::sqrt(66.0), std::sqrt(93.0), std::sqrt(126.0)};

    std::vector<double> ranges = accurate_ri::PointUtils::computeRanges(x, y, z);

    ASSERT_EQ(ranges.size(), expected_ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i) {
        EXPECT_DOUBLE_EQ(ranges[i], expected_ranges[i]);
    }
}

TEST(PointUtilsTest, ComputePhis) {
    std::vector<double> x = {1.0, 2.0, 3.0};
    std::vector<double> z = {7.0, 8.0, 9.0};
    std::vector<double> ranges = {std::sqrt(66.0), std::sqrt(93.0), std::sqrt(126.0)};
    std::vector<double> expected_phis = {std::asin(7.0 / std::sqrt(66.0)), std::asin(8.0 / std::sqrt(93.0)), std::asin(9.0 / std::sqrt(126.0))};

    std::vector<double> phis = accurate_ri::PointUtils::computePhis(x, z, ranges);

    ASSERT_EQ(phis.size(), expected_phis.size());
    for (size_t i = 0; i < phis.size(); ++i) {
        EXPECT_DOUBLE_EQ(phis[i], expected_phis[i]);
    }
}

TEST(PointUtilsTest, ComputeThetas) {
    std::vector<double> x = {1.0, 2.0, 3.0};
    std::vector<double> y = {4.0, 5.0, 6.0};
    std::vector<double> expected_thetas = {std::atan2(4.0, 1.0), std::atan2(5.0, 2.0), std::atan2(6.0, 3.0)};

    std::vector<double> thetas = accurate_ri::PointUtils::computeThetas(x, y);

    ASSERT_EQ(thetas.size(), expected_thetas.size());
    for (size_t i = 0; i < thetas.size(); ++i) {
        EXPECT_DOUBLE_EQ(thetas[i], expected_thetas[i]);
    }
}
