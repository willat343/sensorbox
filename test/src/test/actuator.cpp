#include "sensorbox/actuator.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>

#include "sensorbox/current.hpp"
#include "sensorbox/encoder.hpp"
#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Timestamp = ActuatorMeasurement::Timestamp;

ActuatorMeasurement test_actuator_measurement(const unsigned int i) {
    ActuatorMeasurement measurement{Timestamp{std::chrono::nanoseconds{42}}, test_string(0), test_string(1),
            test_string(i), ActuatorType::SERIES_ELASTIC};
    measurement.joint_position() = 0.1 * i;
    measurement.joint_velocity() = 0.2 * i;
    measurement.joint_torque() = 0.3 * i;
    return measurement;
}

ActuatorMeasurements test_actuator_measurements() {
    return ActuatorMeasurements{Timestamp{std::chrono::nanoseconds{42}},
            std::vector<ActuatorMeasurement>{test_actuator_measurement(0), test_actuator_measurement(1),
                    test_actuator_measurement(2)}};
}

TEST(current_sensor, construction_from_config) {
    const CurrentSensor sensor{test_current_sensor_config()};
    EXPECT_EQ(sensor.type(), SensorType::CURRENT);
    EXPECT_TRUE(sensor.stiffness().isApprox(CurrentSensor::Stiffness{1.0 / 0.05}));
}

TEST(current_sensor, invalid_config_throws) {
    EXPECT_THROW(CurrentSensor(nlohmann::json{{"type", "CURRENT"}}), std::exception);
}

TEST(encoder, construction_from_config) {
    const Encoder encoder{test_encoder_config()};
    EXPECT_EQ(encoder.type(), SensorType::ENCODER);
    EXPECT_TRUE(encoder.stiffness().isApprox(Encoder::Stiffness{1.0 / 0.001}));
    EXPECT_TRUE(encoder.covariance().isApprox(Encoder::Covariance{0.001 * 0.001}));
}

TEST(encoder, invalid_config_throws) {
    EXPECT_THROW(Encoder(nlohmann::json{{"type", "ENCODER"}}), std::exception);
}

TEST(actuator, construction_from_config) {
    const Actuator actuator{test_actuator_config()};
    EXPECT_EQ(actuator.type(), SensorType::ACTUATOR);
    EXPECT_EQ(actuator.actuator_type(), ActuatorType::SERIES_ELASTIC);
    EXPECT_EQ(actuator.make(), "ANYbotics");
    EXPECT_EQ(actuator.model(), "ANYdrive");
    ASSERT_TRUE(actuator.current_sensor().has_value());
    EXPECT_EQ(actuator.current_sensor()->type(), SensorType::CURRENT);
    ASSERT_TRUE(actuator.motor_encoder().has_value());
    EXPECT_EQ(actuator.motor_encoder()->type(), SensorType::ENCODER);
    ASSERT_TRUE(actuator.joint_encoder().has_value());
    EXPECT_EQ(actuator.joint_encoder()->type(), SensorType::ENCODER);
}

TEST(actuator, construction_from_config_without_optional_sensors) {
    nlohmann::json config = test_actuator_config();
    config.erase("current_sensor");
    config.erase("motor_encoder");
    config.erase("joint_encoder");
    const Actuator actuator{config};
    EXPECT_FALSE(actuator.current_sensor().has_value());
    EXPECT_FALSE(actuator.motor_encoder().has_value());
    EXPECT_FALSE(actuator.joint_encoder().has_value());
}

TEST(actuator, invalid_config_throws) {
    nlohmann::json config = test_actuator_config();
    config.erase("actuator_type");
    EXPECT_THROW((Actuator{config}), std::exception);
    config = test_actuator_config();
    config["actuator_type"] = "NOT_AN_ACTUATOR_TYPE";
    EXPECT_THROW((Actuator{config}), std::exception);
}

TEST(actuator_measurement, default_construction) {
    const ActuatorMeasurement measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{ActuatorMeasurement::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
    EXPECT_TRUE(measurement.name().empty());
    EXPECT_EQ(measurement.actuator_type(), ActuatorType::UNSPECIFIED);
    EXPECT_FALSE(measurement.current().has_value());
    EXPECT_FALSE(measurement.joint_position().has_value());
    EXPECT_FALSE(measurement.joint_velocity().has_value());
    EXPECT_FALSE(measurement.joint_torque().has_value());
    EXPECT_FALSE(measurement.motor_position().has_value());
    EXPECT_FALSE(measurement.motor_velocity().has_value());
    EXPECT_FALSE(measurement.motor_torque().has_value());
}

TEST(actuator_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const ActuatorMeasurement measurement{timestamp, test_string(0), test_string(1), test_string(2),
            ActuatorType::GEARED};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    EXPECT_EQ(measurement.name(), test_string(2));
    EXPECT_EQ(measurement.actuator_type(), ActuatorType::GEARED);
}

TEST(actuator_measurement, mutable_access) {
    ActuatorMeasurement measurement;
    measurement.name() = test_string(3);
    measurement.set_actuator_type(ActuatorType::STEPPER);
    measurement.current() = 1.0;
    measurement.joint_position() = 2.0;
    measurement.joint_velocity() = 3.0;
    measurement.joint_torque() = 4.0;
    measurement.motor_position() = 5.0;
    measurement.motor_velocity() = 6.0;
    measurement.motor_torque() = 7.0;
    EXPECT_EQ(measurement.name(), test_string(3));
    EXPECT_EQ(measurement.actuator_type(), ActuatorType::STEPPER);
    EXPECT_DOUBLE_EQ(*measurement.current(), 1.0);
    EXPECT_DOUBLE_EQ(*measurement.joint_position(), 2.0);
    EXPECT_DOUBLE_EQ(*measurement.joint_velocity(), 3.0);
    EXPECT_DOUBLE_EQ(*measurement.joint_torque(), 4.0);
    EXPECT_DOUBLE_EQ(*measurement.motor_position(), 5.0);
    EXPECT_DOUBLE_EQ(*measurement.motor_velocity(), 6.0);
    EXPECT_DOUBLE_EQ(*measurement.motor_torque(), 7.0);
}

TEST(actuator_measurements, default_construction) {
    const ActuatorMeasurements measurements;
    EXPECT_EQ(measurements.timestamp(), Timestamp{ActuatorMeasurements::Duration::zero()});
    EXPECT_EQ(measurements.size(), 0);
    EXPECT_TRUE(measurements.measurements().empty());
}

TEST(actuator_measurements, construction) {
    const ActuatorMeasurements measurements = test_actuator_measurements();
    EXPECT_EQ(measurements.timestamp(), Timestamp{std::chrono::nanoseconds{42}});
    EXPECT_EQ(measurements.size(), 3);
}

TEST(actuator_measurements, joint_quantities) {
    const ActuatorMeasurements measurements = test_actuator_measurements();
    const std::vector<double> expected_positions{0.0, 0.1, 0.2};
    const std::vector<double> expected_velocities{0.0, 0.2, 0.4};
    const std::vector<double> expected_torques{0.0, 0.3, 0.6};
    ASSERT_EQ(measurements.joint_positions().size(), 3);
    for (std::size_t i = 0; i < measurements.size(); ++i) {
        EXPECT_DOUBLE_EQ(measurements.joint_positions()[i], expected_positions[i]);
        EXPECT_DOUBLE_EQ(measurements.joint_velocities()[i], expected_velocities[i]);
        EXPECT_DOUBLE_EQ(measurements.joint_torques()[i], expected_torques[i]);
    }
}

TEST(actuator_measurements, names) {
    const ActuatorMeasurements measurements = test_actuator_measurements();
    const std::vector<std::string> expected_names{test_string(0), test_string(1), test_string(2)};
    EXPECT_EQ(measurements.names(), expected_names);
}

TEST(actuator_measurements, overwrite_names) {
    ActuatorMeasurements measurements = test_actuator_measurements();
    const std::vector<std::string> new_names{test_string(10), test_string(11), test_string(12)};
    measurements.overwrite_names(new_names);
    EXPECT_EQ(measurements.names(), new_names);
}

TEST(actuator_measurements, overwrite_names_of_wrong_size_throws) {
    ActuatorMeasurements measurements = test_actuator_measurements();
    EXPECT_THROW(measurements.overwrite_names(std::vector<std::string>{test_string(10)}), std::runtime_error);
}

TEST(actuator_measurements, equal_joint_measurements) {
    const ActuatorMeasurements measurements = test_actuator_measurements();
    ActuatorMeasurements other = test_actuator_measurements();
    EXPECT_TRUE(measurements.equal_joint_positions(other));
    EXPECT_TRUE(measurements.equal_joint_velocities(other));
    EXPECT_TRUE(measurements.equal_joint_torques(other));
    EXPECT_TRUE(measurements.equal_joint_measurements(other));
    other.measurements()[1].joint_position() = 100.0;
    EXPECT_FALSE(measurements.equal_joint_positions(other));
    EXPECT_TRUE(measurements.equal_joint_velocities(other));
    EXPECT_FALSE(measurements.equal_joint_measurements(other));
    other = test_actuator_measurements();
    other.measurements()[2].joint_torque() = 100.0;
    EXPECT_FALSE(measurements.equal_joint_torques(other));
    EXPECT_FALSE(measurements.equal_joint_measurements(other));
}

TEST(actuator_measurements, equal_joint_measurements_of_different_sizes) {
    const ActuatorMeasurements measurements = test_actuator_measurements();
    ActuatorMeasurements other = test_actuator_measurements();
    other.measurements().pop_back();
    EXPECT_FALSE(measurements.equal_joint_positions(other));
    EXPECT_FALSE(measurements.equal_joint_velocities(other));
    EXPECT_FALSE(measurements.equal_joint_torques(other));
    EXPECT_FALSE(measurements.equal_joint_measurements(other));
}

TEST(actuator_type, string_conversion) {
    for (const ActuatorType actuator_type : ActuatorType::values()) {
        EXPECT_EQ(ActuatorType(std::string(actuator_type)), actuator_type);
    }
}

}
