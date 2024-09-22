#include "sensorbox/imu.hpp"

#include <gtest/gtest.h>

#include <mathbox/lerp.hpp>

#include "sensorbox/test/test_instances.hpp"

TEST(imu, lerp_trivial_0) {
    sensorbox::Imu imu0 = sensorbox::test_imu(0);
    sensorbox::Imu imu1 = sensorbox::test_imu(1);
    sensorbox::Imu imu_interp = sensorbox::lerp(imu0, imu1, imu0.timestamp);
    EXPECT_EQ(imu0, imu_interp);
}

TEST(imu, lerp_trivial_1) {
    sensorbox::Imu imu0 = sensorbox::test_imu(0);
    sensorbox::Imu imu1 = sensorbox::test_imu(1);
    sensorbox::Imu imu_interp = sensorbox::lerp(imu0, imu1, imu1.timestamp);
    EXPECT_EQ(imu1, imu_interp);
}

TEST(imu, lerp_0) {
    sensorbox::Imu imu0 = sensorbox::test_imu(0);
    sensorbox::Imu imu1 = sensorbox::test_imu(1);
    const double interp_true{0.3};
    sensorbox::Imu::TimeStamp interp_timestamp =
            imu0.timestamp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     (imu1.timestamp - imu0.timestamp) * interp_true);
    const double interp = std::chrono::duration<double>(interp_timestamp - imu0.timestamp).count() /
                          std::chrono::duration<double>(imu1.timestamp - imu0.timestamp).count();
    sensorbox::Imu imu_interp = sensorbox::lerp(imu0, imu1, interp_timestamp);
    EXPECT_TRUE(imu_interp.orientation.isApprox(imu0.orientation.slerp(interp, imu1.orientation)));
    EXPECT_TRUE(imu_interp.angular_velocity.isApprox(math::lerp(imu0.angular_velocity, imu1.angular_velocity, interp)));
    EXPECT_TRUE(imu_interp.linear_acceleration.isApprox(
            math::lerp(imu0.linear_acceleration, imu1.linear_acceleration, interp)));
    EXPECT_TRUE(imu_interp.orientation_covariance.isApprox(
            math::lerp(imu0.orientation_covariance, imu1.orientation_covariance, interp)));
    EXPECT_TRUE(imu_interp.angular_velocity_covariance.isApprox(
            math::lerp(imu0.angular_velocity_covariance, imu1.angular_velocity_covariance, interp)));
    EXPECT_TRUE(imu_interp.linear_acceleration_covariance.isApprox(
            math::lerp(imu0.linear_acceleration_covariance, imu1.linear_acceleration_covariance, interp)));
    EXPECT_EQ(imu_interp.frame, imu0.frame);
    EXPECT_EQ(imu_interp.timestamp, interp_timestamp);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
