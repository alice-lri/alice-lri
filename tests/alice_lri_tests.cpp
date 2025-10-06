#include <gtest/gtest.h>
#include "alice_lri/Core.hpp"
#include <vector>

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