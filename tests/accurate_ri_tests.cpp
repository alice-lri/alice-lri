#include <gtest/gtest.h>
#include "accurate_ri/accurate_ri.hpp"
#include <vector>

class AccurateRIAPITest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize test data
    }

    void TearDown() override {
        // Clean up any test state
    }
};

TEST_F(AccurateRIAPITest, ExecuteWithEmptyData) {
    const accurate_ri::PointCloud::Double empty;
    auto result = accurate_ri::train(empty);

    assert(!result.ok());
    assert(result.status().code == accurate_ri::ErrorCode::EMPTY_POINT_CLOUD);
} 