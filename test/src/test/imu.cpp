#include "sensorbox/imu.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Timestamp = ImuMeasurement<3>::Timestamp;

TEST(imu, construction_from_config) {
    const Imu<3> imu{test_imu_config()};
    EXPECT_EQ(imu.type(), SensorType::IMU);
    EXPECT_EQ(imu.make(), "EPSON");
    EXPECT_EQ(imu.model(), "G365PDF1");
    EXPECT_EQ(Imu<3>::D, 3);
    EXPECT_EQ(imu.accelerometer().type(), SensorType::ACCELEROMETER);
    EXPECT_DOUBLE_EQ(imu.accelerometer().frequency(), 400.0);
    EXPECT_DOUBLE_EQ(imu.accelerometer().noise_density(), 5.5e-4);
    EXPECT_EQ(imu.gyroscope().type(), SensorType::GYROSCOPE);
    EXPECT_DOUBLE_EQ(imu.gyroscope().frequency(), 200.0);
    EXPECT_DOUBLE_EQ(imu.gyroscope().noise_density(), 2.0e-5);
}

TEST(imu, invalid_config_throws) {
    nlohmann::json config = test_imu_config();
    config.erase("gyroscope");
    EXPECT_THROW((Imu<3>{config}), std::exception);
}

TEST(imu_measurement, default_construction) {
    const ImuMeasurement<3> measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{ImuMeasurement<3>::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.angular_velocity().isZero());
    EXPECT_TRUE(measurement.linear_acceleration().isZero());
}

TEST(imu_measurement, dimensions) {
    EXPECT_EQ(ImuMeasurement<2>::D, 2);
    EXPECT_EQ(ImuMeasurement<2>::AccelDoF, 2);
    EXPECT_EQ(ImuMeasurement<2>::GyroDoF, 1);
    EXPECT_EQ(ImuMeasurement<3>::D, 3);
    EXPECT_EQ(ImuMeasurement<3>::AccelDoF, 3);
    EXPECT_EQ(ImuMeasurement<3>::GyroDoF, 3);
}

TEST(imu_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{123456789}};
    const ImuMeasurement<3> measurement{timestamp, test_string(0), test_vector3(1), test_vector3(2)};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_TRUE(measurement.angular_velocity().isApprox(test_vector3(1)));
    EXPECT_TRUE(measurement.linear_acceleration().isApprox(test_vector3(2)));
}

TEST(imu_measurement, mutable_access) {
    ImuMeasurement<3> measurement;
    measurement.angular_velocity() = test_vector3(3);
    measurement.linear_acceleration() = test_vector3(4);
    EXPECT_TRUE(measurement.angular_velocity().isApprox(test_vector3(3)));
    EXPECT_TRUE(measurement.linear_acceleration().isApprox(test_vector3(4)));
}

TEST(imu_measurement, transform_to_new_frame_with_identity) {
    const Timestamp timestamp{std::chrono::nanoseconds{5}};
    const ImuMeasurement<3> measurement{timestamp, test_string(0), test_vector3(1), test_vector3(2)};
    const ImuMeasurement<3> transformed =
            measurement.transform_to_new_frame(test_string(5), ImuMeasurement<3>::Pose::Identity());
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(5));
    EXPECT_TRUE(transformed.angular_velocity().isApprox(measurement.angular_velocity()));
    EXPECT_TRUE(transformed.linear_acceleration().isApprox(measurement.linear_acceleration()));
}

TEST(imu_measurement, transform_to_new_frame_with_non_identity_throws) {
    const ImuMeasurement<3> measurement{Timestamp{std::chrono::nanoseconds{5}}, test_string(0), test_vector3(1),
            test_vector3(2)};
    EXPECT_THROW(measurement.transform_to_new_frame(test_string(5), ImuMeasurement<3>::Pose{test_isometry3(1)}),
            std::runtime_error);
}

}
