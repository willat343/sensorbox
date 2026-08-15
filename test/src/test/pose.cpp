#include "sensorbox/pose.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Pose = PoseMeasurement<3>::Pose;
using Timestamp = PoseMeasurement<3>::Timestamp;

TEST(direct_pose_sensor, construction_from_config) {
    const DirectPoseSensor<3> sensor{test_direct_pose_sensor_config()};
    EXPECT_EQ(sensor.type(), SensorType::DIRECT_POSE);
    EXPECT_EQ(DirectPoseSensor<2>::D, 2);
    EXPECT_EQ(DirectPoseSensor<2>::DoF, 3);
    EXPECT_EQ(DirectPoseSensor<3>::D, 3);
    EXPECT_EQ(DirectPoseSensor<3>::DoF, 6);
    const Eigen::Matrix<double, 6, 6> expected_stiffness{
            Eigen::Vector<double, 6>{10.0, 5.0, 10.0 / 3.0, 2.5, 2.0, 10.0 / 6.0}.asDiagonal()};
    EXPECT_TRUE(sensor.stiffness().isApprox(expected_stiffness));
}

TEST(direct_pose_sensor, invalid_config_throws) {
    // Uncertainty is required
    EXPECT_THROW((DirectPoseSensor<3>{nlohmann::json{{"type", "DIRECT_POSE"}}}), std::exception);
    // Wrong number of sigmas for the degrees of freedom
    EXPECT_THROW((DirectPoseSensor<3>{test_direct_position_sensor_config()}), std::exception);
}

TEST(pose_measurement, default_construction) {
    const PoseMeasurement<3> measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{PoseMeasurement<3>::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
    EXPECT_TRUE(measurement.pose().isApprox(Pose::Identity()));
}

TEST(pose_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1)};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    EXPECT_TRUE(measurement.pose().isApprox(test_isometry3(1)));
}

TEST(pose_measurement, mutable_access) {
    PoseMeasurement<3> measurement;
    measurement.pose() = test_isometry3(2);
    EXPECT_TRUE(measurement.pose().isApprox(test_isometry3(2)));
}

TEST(pose_measurement, inverse) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1)};
    const PoseMeasurement<3> inverse = measurement.inverse();
    EXPECT_EQ(inverse.timestamp(), timestamp);
    EXPECT_EQ(inverse.frame(), test_string(1));
    EXPECT_EQ(inverse.child_frame(), test_string(0));
    EXPECT_TRUE(inverse.pose().isApprox(test_isometry3(1).inverse()));
    EXPECT_TRUE(inverse.inverse().pose().isApprox(measurement.pose()));
}

TEST(pose_measurement, invert) {
    PoseMeasurement<3> measurement{Timestamp{std::chrono::nanoseconds{42}}, test_string(0), test_string(1),
            test_isometry3(1)};
    const PoseMeasurement<3> inverse = measurement.inverse();
    measurement.invert();
    EXPECT_EQ(measurement.frame(), inverse.frame());
    EXPECT_EQ(measurement.child_frame(), inverse.child_frame());
    EXPECT_TRUE(measurement.pose().isApprox(inverse.pose()));
}

TEST(pose_measurement, transform_to_new_child_frame) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1)};
    const Pose T_C_N{test_isometry3(2)};
    const PoseMeasurement<3> transformed = measurement.transform_to_new_child_frame(test_string(2), T_C_N);
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(0));
    EXPECT_EQ(transformed.child_frame(), test_string(2));
    EXPECT_TRUE(transformed.pose().isApprox(test_isometry3(1) * T_C_N));
}

TEST(pose_measurement, transform_to_new_frame) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1)};
    const Pose T_N_F{test_isometry3(3)};
    const PoseMeasurement<3> transformed = measurement.transform_to_new_frame(test_string(3), T_N_F);
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(3));
    EXPECT_EQ(transformed.child_frame(), test_string(1));
    EXPECT_TRUE(transformed.pose().isApprox(T_N_F * test_isometry3(1)));
}

TEST(pose_measurement, transform_to_new_frames) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1)};
    const Pose T_NF_F{test_isometry3(3)};
    const Pose T_C_NC{test_isometry3(2)};
    const PoseMeasurement<3> transformed =
            measurement.transform_to_new_frames(test_string(3), test_string(2), T_NF_F, T_C_NC);
    EXPECT_EQ(transformed.timestamp(), timestamp);
    EXPECT_EQ(transformed.frame(), test_string(3));
    EXPECT_EQ(transformed.child_frame(), test_string(2));
    EXPECT_TRUE(transformed.pose().isApprox(T_NF_F * test_isometry3(1) * T_C_NC));
    // Applying the frame changes separately must give the same result
    EXPECT_TRUE(transformed.pose().isApprox(measurement.transform_to_new_frame(test_string(3), T_NF_F)
                    .transform_to_new_child_frame(test_string(2), T_C_NC)
                    .pose()));
}

TEST(pose_measurements, default_construction) {
    const PoseMeasurements<3> measurements;
    EXPECT_EQ(measurements.timestamp(), Timestamp{PoseMeasurements<3>::Duration::zero()});
    EXPECT_TRUE(measurements.measurements().empty());
}

TEST(pose_measurements, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const std::vector<PoseMeasurement<3>> measurement_vector{
            PoseMeasurement<3>{timestamp, test_string(0), test_string(1), test_isometry3(1)},
            PoseMeasurement<3>{timestamp, test_string(0), test_string(2), test_isometry3(2)}};
    const PoseMeasurements<3> measurements{timestamp, measurement_vector};
    EXPECT_EQ(measurements.timestamp(), timestamp);
    ASSERT_EQ(measurements.measurements().size(), 2);
    EXPECT_EQ(measurements.measurements()[1].child_frame(), test_string(2));
    EXPECT_TRUE(measurements.measurements()[1].pose().isApprox(test_isometry3(2)));
}

TEST(pose_measurements, construction_with_timestamp_only) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const PoseMeasurements<3> measurements{timestamp};
    EXPECT_EQ(measurements.timestamp(), timestamp);
    EXPECT_TRUE(measurements.measurements().empty());
}

TEST(pose_measurements, mutable_access) {
    PoseMeasurements<3> measurements;
    measurements.measurements().emplace_back();
    EXPECT_EQ(measurements.measurements().size(), 1);
}

}
