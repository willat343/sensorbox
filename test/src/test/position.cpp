#include "sensorbox/position.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Pose = PositionMeasurement<3>::Pose;
using RotationMatrix = PositionMeasurement<3>::RotationMatrix;
using Timestamp = PositionMeasurement<3>::Timestamp;

TEST(direct_position_sensor, construction_from_config) {
    const DirectPositionSensor<3> sensor{test_direct_position_sensor_config()};
    EXPECT_EQ(sensor.type(), SensorType::DIRECT_POSITION);
    EXPECT_EQ(DirectPositionSensor<2>::D, 2);
    EXPECT_EQ(DirectPositionSensor<2>::DoF, 2);
    EXPECT_EQ(DirectPositionSensor<3>::D, 3);
    EXPECT_EQ(DirectPositionSensor<3>::DoF, 3);
    const Eigen::Matrix3d expected_stiffness{Eigen::Vector3d{10.0, 5.0, 10.0 / 3.0}.asDiagonal()};
    EXPECT_TRUE(sensor.stiffness().isApprox(expected_stiffness));
}

TEST(direct_position_sensor, invalid_config_throws) {
    // Uncertainty is required
    EXPECT_THROW((DirectPositionSensor<3>{nlohmann::json{{"type", "DIRECT_POSITION"}}}), std::exception);
    // Wrong number of sigmas for the degrees of freedom
    EXPECT_THROW((DirectPositionSensor<3>{test_direct_pose_sensor_config()}), std::exception);
}

TEST(position_measurement, default_construction) {
    const PositionMeasurement<3> measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{PositionMeasurement<3>::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
    EXPECT_TRUE(measurement.position().isZero());
}

TEST(position_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PositionMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(1)};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    EXPECT_TRUE(measurement.position().isApprox(test_vector3(1)));
}

TEST(position_measurement, mutable_access) {
    PositionMeasurement<3> measurement;
    measurement.position() = test_vector3(2);
    EXPECT_TRUE(measurement.position().isApprox(test_vector3(2)));
}

TEST(position_measurement, inverse) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PositionMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(1)};
    const RotationMatrix R_C_F = test_quaternion(1).toRotationMatrix();
    const PositionMeasurement<3> inverse = measurement.inverse(R_C_F);
    EXPECT_EQ(inverse.timestamp(), timestamp);
    EXPECT_EQ(inverse.frame(), test_string(1));
    EXPECT_EQ(inverse.child_frame(), test_string(0));
    EXPECT_TRUE(inverse.position().isApprox(-R_C_F * test_vector3(1)));
    // Inverting twice with the inverse rotation recovers the original measurement
    EXPECT_TRUE(inverse.inverse(R_C_F.transpose()).position().isApprox(measurement.position()));
}

TEST(position_measurement, invert) {
    PositionMeasurement<3> measurement{Timestamp{std::chrono::nanoseconds{42}}, test_string(0), test_string(1),
            test_vector3(1)};
    const RotationMatrix R_C_F = test_quaternion(1).toRotationMatrix();
    const PositionMeasurement<3> inverse = measurement.inverse(R_C_F);
    measurement.invert(R_C_F);
    EXPECT_EQ(measurement.frame(), inverse.frame());
    EXPECT_EQ(measurement.child_frame(), inverse.child_frame());
    EXPECT_TRUE(measurement.position().isApprox(inverse.position()));
}

TEST(position_measurement, transform_to_new_child_frame) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PositionMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(1)};
    const RotationMatrix R_F_C = test_quaternion(2).toRotationMatrix();
    const PositionMeasurement<3> transformed =
            measurement.transform_to_new_child_frame(test_string(2), R_F_C, test_vector3(3));
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(0));
    EXPECT_EQ(transformed.child_frame(), test_string(2));
    EXPECT_TRUE(transformed.position().isApprox(test_vector3(1) + R_F_C * test_vector3(3)));
}

TEST(position_measurement, transform_to_new_frame) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PositionMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(1)};
    const Pose T_N_F{test_isometry3(2)};
    const PositionMeasurement<3> transformed = measurement.transform_to_new_frame(test_string(2), T_N_F);
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(2));
    EXPECT_EQ(transformed.child_frame(), test_string(1));
    EXPECT_TRUE(transformed.position().isApprox(T_N_F * test_vector3(1)));
    // The transformation is affine, i.e. it applies the rotation and the translation
    EXPECT_TRUE(transformed.position().isApprox(T_N_F.rotation() * test_vector3(1) + T_N_F.translation()));
}

TEST(position_measurement, transform_to_new_frames) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PositionMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(1)};
    const Pose T_NF_F{test_isometry3(2)};
    const RotationMatrix R_F_C = test_quaternion(3).toRotationMatrix();
    const PositionMeasurement<3> transformed =
            measurement.transform_to_new_frames(test_string(2), test_string(3), T_NF_F, R_F_C, test_vector3(4));
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(2));
    EXPECT_EQ(transformed.child_frame(), test_string(3));
    EXPECT_TRUE(transformed.position().isApprox(T_NF_F * (test_vector3(1) + R_F_C * test_vector3(4))));
    // Applying the frame changes separately must give the same result
    EXPECT_TRUE(transformed.position().isApprox(
            measurement.transform_to_new_child_frame(test_string(3), R_F_C, test_vector3(4))
                    .transform_to_new_frame(test_string(2), T_NF_F)
                    .position()));
}

}
