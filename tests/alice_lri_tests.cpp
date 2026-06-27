#include <gtest/gtest.h>
#include "alice_lri/Core.hpp"
#include <cmath>
#include <limits>
#include <vector>

namespace {
    alice_lri::Intrinsics buildSingleScanlineIntrinsics() {
        alice_lri::Intrinsics intrinsics(1);
        intrinsics.scanlines[0].verticalOffset = 0.0;
        intrinsics.scanlines[0].verticalAngle = 0.0;
        intrinsics.scanlines[0].horizontalOffset = 0.0;
        intrinsics.scanlines[0].azimuthalOffset = 0.0;
        intrinsics.scanlines[0].resolution = 4;
        return intrinsics;
    }

    alice_lri::PointCloud::Double buildCardinalPointCloud() {
        alice_lri::PointCloud::Double points;
        points.x = alice_lri::AliceArray<double>{-1.0, 0.0, 1.0, 0.0};
        points.y = alice_lri::AliceArray<double>{0.0, -1.0, 0.0, 1.0};
        points.z = alice_lri::AliceArray<double>{0.0, 0.0, 0.0, 0.0};
        return points;
    }
}

class ALICELRIAPITest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize test data
    }

    void TearDown() override {
        // Clean up any test state
    }
};

TEST_F(ALICELRIAPITest, ExecuteWithEmptyData) {
    const alice_lri::PointCloud::Double empty;
    auto result = alice_lri::estimateIntrinsics(empty);

    assert(!result.ok());
    assert(result.status().code == alice_lri::ErrorCode::EMPTY_POINT_CLOUD);
}

TEST_F(ALICELRIAPITest, ProjectToRangeImageUsesCustomEmptyValue) {
    const auto intrinsics = buildSingleScanlineIntrinsics();
    alice_lri::PointCloud::Double points;
    points.x = alice_lri::AliceArray<double>{-1.0, 1.0};
    points.y = alice_lri::AliceArray<double>{0.0, 0.0};
    points.z = alice_lri::AliceArray<double>{0.0, 0.0};

    const auto result = alice_lri::projectToRangeImage(intrinsics, points, -1.0);

    ASSERT_TRUE(result.ok());
    const auto &image = result.value();
    EXPECT_EQ(image.width(), 4);
    EXPECT_EQ(image.height(), 1);
    EXPECT_DOUBLE_EQ(image(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(image(0, 1), -1.0);
    EXPECT_DOUBLE_EQ(image(0, 2), 1.0);
    EXPECT_DOUBLE_EQ(image(0, 3), -1.0);
}

TEST_F(ALICELRIAPITest, ProjectValuesToRangeImagePlacesCustomScalars) {
    const auto intrinsics = buildSingleScanlineIntrinsics();
    const auto points = buildCardinalPointCloud();
    const alice_lri::AliceArray<double> values{10.0, 20.0, 30.0, 40.0};

    const auto result = alice_lri::projectValuesToRangeImage(intrinsics, points, values);

    ASSERT_TRUE(result.ok());
    const auto &image = result.value();
    EXPECT_EQ(image.width(), 4);
    EXPECT_EQ(image.height(), 1);
    EXPECT_DOUBLE_EQ(image(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(image(0, 1), 20.0);
    EXPECT_DOUBLE_EQ(image(0, 2), 30.0);
    EXPECT_DOUBLE_EQ(image(0, 3), 40.0);
}

TEST_F(ALICELRIAPITest, ProjectValuesToRangeImageRejectsMismatchedValues) {
    const auto intrinsics = buildSingleScanlineIntrinsics();
    const auto points = buildCardinalPointCloud();
    const alice_lri::AliceArray<double> values{10.0, 20.0, 30.0};

    const auto result = alice_lri::projectValuesToRangeImage(intrinsics, points, values);

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.status().code, alice_lri::ErrorCode::MISMATCHED_SIZES);
}

TEST_F(ALICELRIAPITest, ProjectValuesToRangeImageSupportsNaNEmptyValue) {
    const auto intrinsics = buildSingleScanlineIntrinsics();
    alice_lri::PointCloud::Double points;
    points.x = alice_lri::AliceArray<double>{-1.0, 1.0};
    points.y = alice_lri::AliceArray<double>{0.0, 0.0};
    points.z = alice_lri::AliceArray<double>{0.0, 0.0};
    const alice_lri::AliceArray<double> values{10.0, 30.0};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    const auto result = alice_lri::projectValuesToRangeImage(intrinsics, points, values, nan);

    ASSERT_TRUE(result.ok());
    const auto &image = result.value();
    EXPECT_DOUBLE_EQ(image(0, 0), 10.0);
    EXPECT_TRUE(std::isnan(image(0, 1)));
    EXPECT_DOUBLE_EQ(image(0, 2), 30.0);
    EXPECT_TRUE(std::isnan(image(0, 3)));
}
