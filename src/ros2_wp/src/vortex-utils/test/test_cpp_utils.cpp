#include <gtest/gtest.h>
#include <vortex_utils/cpp_utils.hpp>

namespace vortex_utils {

// Test that the value does not change when already in the interval [-pi, pi]
TEST(ssa, test_ssa_0) {
    EXPECT_EQ(0, ssa(0));
}

// Test that 2 pi correctly maps to 0
TEST(ssa, test_ssa_2pi) {
    EXPECT_EQ(0, ssa(2 * M_PI));
}

// Test that values over pi gets mapped to the negative interval
TEST(ssa, test_ssa_3_5) {
    EXPECT_NEAR(-2.78, ssa(3.5), 0.01);
}

// Test that values under -pi gets mapped to the positive interval
TEST(ssa, test_ssa_minus_3_5) {
    EXPECT_NEAR(2.78, ssa(-3.5), 0.01);
}

// Test that the skew-symmetric matrix is correctly calculated
TEST(skew_symmetric, test_skew_symmetric) {
    Eigen::Vector3d vector(1, 2, 3);
    Eigen::Matrix3d expected;
    expected << 0, -3, 2, 3, 0, -1, -2, 1, 0;

    Eigen::Matrix3d result = skew_symmetric(vector);
    EXPECT_EQ(expected, result);
}

// Test that the identity quaternion correctly maps to 0, 0, 0
TEST(quat_to_euler, test_quat_to_euler_1) {
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d expected(0.0, 0.0, 0.0);
    Eigen::Vector3d result = quat_to_euler(q);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 0.01);
    }
}

// Test that only changing w and x in the quat only affects roll
TEST(quat_to_euler, test_quat_to_euler_2) {
    Eigen::Quaterniond q2(0.707, 0.707, 0.0, 0.0);
    Eigen::Vector3d expected2(1.57, 0.0, 0.0);
    Eigen::Vector3d result2 = quat_to_euler(q2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected2[i], result2[i], 0.01);
    }
}

// Test that only changing w and y in the quat only affects pitch
TEST(quat_to_euler, test_quat_to_euler_3) {
    Eigen::Quaterniond q3(0.707, 0.0, 0.707, 0.0);
    Eigen::Vector3d expected3(0.0, 1.57, 0.0);
    Eigen::Vector3d result3 = quat_to_euler(q3);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected3[i], result3[i], 0.01);
    }
}

// Test that only changing w and z in the quat only affects yaw
TEST(quat_to_euler, test_quat_to_euler_4) {
    Eigen::Quaterniond q4(0.707, 0.0, 0.0, 0.707);
    Eigen::Vector3d expected4(0.0, 0.0, 1.57);
    Eigen::Vector3d result4 = quat_to_euler(q4);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected4[i], result4[i], 0.01);
    }
}

TEST(quat_to_euler, test_quat_to_euler_5) {
    Eigen::Quaterniond q5(0.770, 0.4207, -0.4207, -0.229);
    Eigen::Vector3d expected5(1.0, -1.0, 0.0);
    Eigen::Vector3d result5 = quat_to_euler(q5);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected5[i], result5[i], 0.01);
    }
}

}  // namespace vortex_utils

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
