#include <gtest/gtest.h>
#include "point/PointArray.h"
#include <Eigen/Core>
#include <cmath>

namespace accurate_ri {

class PointArrayTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test data
        x = Eigen::ArrayXd(3);
        y = Eigen::ArrayXd(3);
        z = Eigen::ArrayXd(3);
        
        x << 1.0, 2.0, 3.0;
        y << 4.0, 5.0, 6.0;
        z << 7.0, 8.0, 9.0;
    }

    Eigen::ArrayXd x;
    Eigen::ArrayXd y;
    Eigen::ArrayXd z;
};

TEST_F(PointArrayTest, Construction) {
    // Test construction from Eigen arrays
    PointArray points(x, y, z);
    
    EXPECT_EQ(points.size(), 3);
    EXPECT_EQ(points.getX(0), 1.0);
    EXPECT_EQ(points.getY(1), 5.0);
    EXPECT_EQ(points.getZ(2), 9.0);
}

TEST_F(PointArrayTest, PolarConversion) {
    // Test conversion to polar coordinates
    PointArray points(x, y, z);
    
    // Point 0: (1.0, 4.0, 7.0)
    double expectedRange = std::sqrt(1.0*1.0 + 4.0*4.0 + 7.0*7.0);
    double expectedPhi = std::atan2(7.0, std::sqrt(1.0*1.0 + 4.0*4.0));
    
    EXPECT_NEAR(points.getRange(0), expectedRange, 1e-10);
    EXPECT_NEAR(points.getPhi(0), expectedPhi, 1e-10);
    
    // Check array accessors
    EXPECT_EQ(points.getRanges().size(), 3);
    EXPECT_EQ(points.getPhis().size(), 3);
    EXPECT_NEAR(points.getRanges()(0), expectedRange, 1e-10);
    EXPECT_NEAR(points.getPhis()(0), expectedPhi, 1e-10);
}

TEST_F(PointArrayTest, IndexAccess) {
    // Test access via indices
    PointArray points(x, y, z);
    
    Eigen::ArrayXi indices(2);
    indices << 0, 2;
    
    Eigen::ArrayXd expectedX(2);
    expectedX << 1.0, 3.0;
    
    Eigen::ArrayXd selectedX = points.getX()(indices);
    EXPECT_EQ(selectedX.size(), 2);
    EXPECT_NEAR(selectedX(0), expectedX(0), 1e-10);
    EXPECT_NEAR(selectedX(1), expectedX(1), 1e-10);
}

// Add more tests for PointArray functionality

} // namespace accurate_ri 