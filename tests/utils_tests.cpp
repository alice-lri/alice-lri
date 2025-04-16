#include <gtest/gtest.h>
#include "utils/Utils.h"
#include <Eigen/Core>
#include <vector>

namespace accurate_ri {
    class UtilsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            // Initialize test data
            array = Eigen::ArrayXd(5);
            array << 1.0, 3.0, 5.0, 7.0, 9.0; // Example array
        }

        Eigen::ArrayXd array;
    };

    TEST_F(UtilsTest, CompareFunction) {
        // Test compare function
        EXPECT_EQ(Utils::compare(3, 5), -1);
        EXPECT_EQ(Utils::compare(5, 3), 1);
        EXPECT_EQ(Utils::compare(5, 5), 0);
    }

    TEST_F(UtilsTest, SignFunction) {
        // Test sign function
        EXPECT_EQ(Utils::sign(-10), -1);
        EXPECT_EQ(Utils::sign(0), 0);
        EXPECT_EQ(Utils::sign(10), 1);
    }

    TEST_F(UtilsTest, DiffFunction) {
        // Test diff function
        Eigen::ArrayXd diffResult = Utils::diff(array);
        Eigen::ArrayXd expectedDiff(4);
        expectedDiff << 2.0, 2.0, 2.0, 2.0;

        EXPECT_TRUE(diffResult.isApprox(expectedDiff, 1e-10));
    }

    TEST_F(UtilsTest, MedianInPlaceFunction) {
        // Test medianInPlace function
        Eigen::ArrayXd testArray(5);
        testArray << 9.0, 1.0, 5.0, 3.0, 7.0;

        double median = Utils::medianInPlace(testArray);
        EXPECT_NEAR(median, 5.0, 1e-10);
    }

    TEST_F(UtilsTest, EigenMaskToIndicesFunction) {
        // Test eigenMaskToIndices function
        Eigen::ArrayX<bool> mask(5);
        mask << true, false, true, false, true;

        Eigen::ArrayXi indices = Utils::eigenMaskToIndices(mask);
        Eigen::ArrayXi expectedIndices(3);
        expectedIndices << 0, 2, 4;

        EXPECT_EQ(indices.size(), expectedIndices.size());
        EXPECT_TRUE(indices.isApprox(expectedIndices));
    }
} // namespace accurate_ri
