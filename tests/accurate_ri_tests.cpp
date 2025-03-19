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

TEST_F(AccurateRIAPITest, HelloFunctionWorks) {
    // This is a simple test for the hello function
    accurate_ri::hello();
    // No assertion needed - just checking it doesn't crash
}

TEST_F(AccurateRIAPITest, PathHandling) {
    const std::string testPath = "/test/path/to/cloud";
    accurate_ri::setCloudPath(testPath);
    EXPECT_EQ(accurate_ri::getCloudPath(), testPath);
    
    const std::optional<std::string> outputPath = "/test/output/path";
    accurate_ri::setOutputPath(outputPath);
    EXPECT_EQ(accurate_ri::getOutputPath(), outputPath);
}

TEST_F(AccurateRIAPITest, ResidualThreshold) {
    const std::optional<double> threshold = 0.05;
    accurate_ri::setResidualThreshold(threshold);
    EXPECT_EQ(accurate_ri::getResidualThreshold(), threshold);
}

TEST_F(AccurateRIAPITest, ExecuteWithEmptyData) {
    std::vector<float> emptyX, emptyY, emptyZ;
    // Expect no crash with empty data
    accurate_ri::IntrinsicsResult result = accurate_ri::execute(emptyX, emptyY, emptyZ);
    
    // Basic checks on result structure
    EXPECT_EQ(result.vertical.pointsCount, 0);
    EXPECT_EQ(result.vertical.scanlinesCount, 0);
} 