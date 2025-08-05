#include <gtest/gtest.h>
#include "math/Stats.h"
#include <Eigen/Core>
#include <vector>

namespace accurate_ri {

class StatsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize test data
        x = Eigen::ArrayXd(5);
        y = Eigen::ArrayXd(5);
        weights = Eigen::ArrayXd(5);
        
        x << 1.0, 2.0, 3.0, 4.0, 5.0;
        y << 2.0, 4.0, 6.0, 8.0, 10.0; // y = 2x
        weights << 1.0, 1.0, 1.0, 1.0, 1.0;
    }

    Eigen::ArrayXd x;
    Eigen::ArrayXd y;
    Eigen::ArrayXd weights;
};

TEST_F(StatsTest, SimpleLinearRegression) {
    // Test simple linear regression on perfect linear data
    Stats::LRResult result = Stats::linearRegression(x, y);
    
    // For y = 2x, slope should be 2 and intercept should be 0
    EXPECT_NEAR(result.slope, 2.0, 1e-10);
    EXPECT_NEAR(result.intercept, 0.0, 1e-10);
}

TEST_F(StatsTest, WeightedLinearRegression) {
    // Test weighted linear regression
    Stats::WLSResult result = Stats::wlsBoundsFit(x, y, weights);
    
    // For y = 2x with uniform weights, should get the same results as simple LR
    EXPECT_NEAR(result.slope, 2.0, 1e-10);
    EXPECT_NEAR(result.intercept, 0.0, 1e-10);
    
    // Check that results include variance and AIC
    EXPECT_GE(result.slopeVariance, 0.0);
    EXPECT_GE(result.interceptVariance, 0.0);
    EXPECT_FALSE(std::isnan(result.aic));
}

TEST_F(StatsTest, WeightedBoundsRegression) {
    // Create bounds array
    Eigen::ArrayXd bounds = Eigen::ArrayXd::Constant(5, 0.1); // 0.1 error bound for each point
    
    // Test weighted bounds regression
    auto result = Stats::wlsBoundsFit(x, y, bounds);
    
    // For y = 2x with small bounds, should get approximately the same results
    EXPECT_NEAR(result.slope, 2.0, 0.1);
    EXPECT_NEAR(result.intercept, 0.0, 0.1);
    
    // Check confidence intervals
    EXPECT_LE(result.slopeCi(0), result.slope);
    EXPECT_GE(result.slopeCi(1), result.slope);
    EXPECT_LE(result.interceptCi(0), result.intercept);
    EXPECT_GE(result.interceptCi(1), result.intercept);
}

TEST_F(StatsTest, IntMode) {
    // Test the mode computation for integers
    std::vector<int> values = {1, 2, 2, 3, 2, 4, 5};
    int mode = Stats::intMode(values);
    EXPECT_EQ(mode, 2);
    
    // Empty vector case
    std::vector<int> emptyValues;
    mode = Stats::intMode(emptyValues);
    EXPECT_EQ(mode, 0);
}

} // namespace accurate_ri 