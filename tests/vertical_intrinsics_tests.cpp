#include <gtest/gtest.h>
#include "intrinsics/vertical/VerticalIntrinsicsEstimator.h"
#include "point/PointArray.h"
#include <cmath>

namespace accurate_ri {

class VerticalIntrinsicsEstimatorTest : public ::testing::Test {
protected:
    // Add default constructor and initialize testPoints with empty arrays
    VerticalIntrinsicsEstimatorTest() : testPoints(Eigen::ArrayXd(0), Eigen::ArrayXd(0), Eigen::ArrayXd(0)) {}
    
    void SetUp() override {
        // Generate test data that simulates a LiDAR scan with points arranged in vertical scanlines
        const int numPoints = 100; // Reduced for faster testing
        const int numScanlines = 5;
        
        Eigen::ArrayXd x(numPoints);
        Eigen::ArrayXd y(numPoints);
        Eigen::ArrayXd z(numPoints);
        
        // Create synthetic data with known vertical scanline patterns
        for (int i = 0; i < numPoints; i++) {
            int scanline = i % numScanlines;
            double angle = -M_PI/4 + (scanline * M_PI/2) / numScanlines;
            double r = 10.0 + ((double)rand() / RAND_MAX) * 5.0;
            
            x(i) = r * cos(angle);
            y(i) = r * sin(angle);
            z(i) = ((double)rand() / RAND_MAX) * 2.0 - 1.0; // Random z between -1 and 1
        }
        
        testPoints = PointArray(x, y, z);
    }

    PointArray testPoints;
    VerticalIntrinsicsEstimator estimator;
};

TEST_F(VerticalIntrinsicsEstimatorTest, EstimateReturnsValidResult) {
    // Test that the estimator returns a valid result structure
    VerticalIntrinsicsEstimation result = estimator.estimate(testPoints);
    
    // Basic checks on the result structure
    EXPECT_GT(result.pointsCount, 0);
    EXPECT_LE(result.unassignedPoints, result.pointsCount);
    
    // Check that we have an end reason
    EXPECT_TRUE(result.endReason == EndReason::ALL_ASSIGNED || 
                result.endReason == EndReason::MAX_ITERATIONS || 
                result.endReason == EndReason::NO_MORE_PEAKS);
}

TEST_F(VerticalIntrinsicsEstimatorTest, ResultContainsScanlines) {
    // Test that the result contains scanline information
    VerticalIntrinsicsEstimation result = estimator.estimate(testPoints);
    
    // Check that each scanline has valid angle/offset values
    for (const auto& scanline : result.scanlinesAssignations.scanlines) {
        // Offset and angle should be finite values
        EXPECT_FALSE(std::isnan(scanline.offset.value));
        EXPECT_FALSE(std::isinf(scanline.offset.value));
        EXPECT_FALSE(std::isnan(scanline.angle.value));
        EXPECT_FALSE(std::isinf(scanline.angle.value));
        
        // Confidence intervals should make sense
        EXPECT_LE(scanline.offset.ci.lower, scanline.offset.value);
        EXPECT_GE(scanline.offset.ci.upper, scanline.offset.value);
        EXPECT_LE(scanline.angle.ci.lower, scanline.angle.value);
        EXPECT_GE(scanline.angle.ci.upper, scanline.angle.value);
    }
}

TEST_F(VerticalIntrinsicsEstimatorTest, PointsAssignedToScanlines) {
    // Test that points are assigned to scanlines
    VerticalIntrinsicsEstimation result = estimator.estimate(testPoints);
    
    // Check that there's a scanline ID for each point
    EXPECT_EQ(result.scanlinesAssignations.pointsScanlinesIds.size(), testPoints.size());
    
    // Check that all scanline IDs are valid
    std::unordered_set<int> validIds;
    for (const auto& scanline : result.scanlinesAssignations.scanlines) {
        validIds.insert(scanline.id);
    }
    
    for (const auto& pointScanlineId : result.scanlinesAssignations.pointsScanlinesIds) {
        if (pointScanlineId >= 0) { // -1 means unassigned
            EXPECT_TRUE(validIds.find(pointScanlineId) != validIds.end());
        }
    }
}

} // namespace accurate_ri 