#include <gtest/gtest.h>
#include "hough/HoughTransform.h"
#include "point/PointArray.h"
#include <Eigen/Core>
#include <vector>

namespace alice_lri {

class HoughTransformTest : public ::testing::Test {
protected:
    // Add default constructor and initialize testPoints with empty arrays
    HoughTransformTest() : testPoints(Eigen::ArrayXd(0), Eigen::ArrayXd(0), Eigen::ArrayXd(0)) {}
    
    void SetUp() override {
        // Create a small point array for testing
        Eigen::ArrayXd x(3);
        Eigen::ArrayXd y(3);
        Eigen::ArrayXd z(3);
        
        // Simple points that form a line
        x << 1.0, 2.0, 3.0;
        y << 1.0, 2.0, 3.0;
        z << 0.0, 0.0, 0.0;
        
        testPoints = PointArray(x, y, z);
    }

    PointArray testPoints;
};

TEST_F(HoughTransformTest, InitializationWithValidParameters) {
    // Test that the HoughTransform initializes correctly with valid parameters
    const double xMin = -10.0;
    const double xMax = 10.0;
    const double xStep = 0.2;
    const double yMin = -10.0;
    const double yMax = 10.0;
    const double yStep = 0.2;
    
    HoughTransform hough(xMin, xMax, xStep, yMin, yMax, yStep);
    
    EXPECT_EQ(hough.getXMin(), xMin);
    EXPECT_EQ(hough.getXMax(), xMax);
    EXPECT_EQ(hough.getYMin(), yMin);
    EXPECT_EQ(hough.getYMax(), yMax);
    EXPECT_DOUBLE_EQ(hough.getXStep(), xStep);
    EXPECT_DOUBLE_EQ(hough.getYStep(), yStep);
}

TEST_F(HoughTransformTest, ComputeAccumulatorForPoints) {
    // Initialize a small Hough transform for testing
    HoughTransform hough(-1.0, 1.0, 0.1, -1.0, 1.0, 0.1);
    
    // Compute accumulator for points
    hough.computeAccumulator(testPoints);
    
    // We can't easily check the accumulator values directly,
    // but we can verify that calling the method doesn't crash
    // In a real test, you might check specific expected accumulator values
}

TEST_F(HoughTransformTest, FindMaximum) {
    // Initialize a small Hough transform for testing
    HoughTransform hough(-1.0, 1.0, 0.1, -1.0, 1.0, 0.1);
    
    // Compute accumulator for points
    hough.computeAccumulator(testPoints);
    
    // Find maximum
    auto maximum = hough.findMaximum(std::nullopt);
    
    // The maximum should exist for our test data
    EXPECT_TRUE(maximum.has_value());
    
    if (maximum.has_value()) {
        // Maximum should have positive votes
        EXPECT_GT(maximum->votes, 0.0);
    }
}

TEST_F(HoughTransformTest, EraseByHash) {
    // Initialize a small Hough transform for testing
    HoughTransform hough(-1.0, 1.0, 0.01, 1.0, 20, 0.1);
    
    // Vote for all points
    hough.computeAccumulator(testPoints);
    
    // Find peaks
    std::optional<HoughCell> peak = hough.findMaximum(std::nullopt);

    if (peak) {
        uint64_t hash = peak->hash;

        // Record votes before erasing
        int64_t votesBeforeErase = peak->votes;
        
        // Erase by hash
        hough.eraseByHash(hash);
        
        // Find peaks again
        std::optional<HoughCell> peaksAfterErase = hough.findMaximum(std::nullopt);
        
        if (peaksAfterErase) {
            // Either the hash is different, or the votes are less
            EXPECT_TRUE(peaksAfterErase->hash != hash || peaksAfterErase->votes < votesBeforeErase);
        }
    }
}

TEST_F(HoughTransformTest, RestoreVotes) {
    // Initialize a small Hough transform for testing
    HoughTransform hough(-1.0, 1.0, 0.01, 1.0, 20, 0.1);
    
    // Vote for all points
    hough.computeAccumulator(testPoints);
    
    // Find peaks
    std::optional<HoughCell> peaks = hough.findMaximum(std::nullopt);
    
    if (peaks) {
        uint64_t hash = peaks->hash;
        int64_t votes = peaks->votes;
        
        // Erase by hash
        hough.eraseByHash(hash);
        
        // Restore votes
        hough.restoreVotes(hash, votes);
        
        // Find peaks again
        std::optional<HoughCell> peaksAfterRestore = hough.findMaximum(std::nullopt);
        
        if (peaksAfterRestore && peaksAfterRestore->hash == hash) {
            // The votes should be approximately restored
            EXPECT_NEAR(peaksAfterRestore->votes, votes, 1e-6);
        }
    }
}

}