#include "sensorbox/random_walk_sensor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "sensorbox/accelerometer.hpp"
#include "sensorbox/gyroscope.hpp"
#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

TEST(random_walk_sensor, construction) {
    const RandomWalkSensor<3> sensor{SensorType::ACCELEROMETER, 400.0, 5.5e-4, 4.9e-5, 4.9e-2};
    EXPECT_EQ(sensor.type(), SensorType::ACCELEROMETER);
    EXPECT_DOUBLE_EQ(sensor.frequency(), 400.0);
    EXPECT_DOUBLE_EQ(sensor.noise_density(), 5.5e-4);
    EXPECT_DOUBLE_EQ(sensor.bias_noise_density(), 4.9e-5);
    EXPECT_DOUBLE_EQ(sensor.initial_noise(), 4.9e-2);
    EXPECT_DOUBLE_EQ(sensor.period(), 1.0 / 400.0);
    EXPECT_TRUE(sensor.inverse_scaling_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(random_walk_sensor, construction_from_config) {
    const RandomWalkSensor<3> sensor{test_accelerometer_config()};
    EXPECT_EQ(sensor.type(), SensorType::ACCELEROMETER);
    EXPECT_EQ(sensor.make(), "EPSON");
    EXPECT_EQ(sensor.model(), "G365PDF1");
    EXPECT_DOUBLE_EQ(sensor.frequency(), 400.0);
    EXPECT_DOUBLE_EQ(sensor.noise_density(), 5.5e-4);
    EXPECT_DOUBLE_EQ(sensor.bias_noise_density(), 4.903325e-5);
    EXPECT_DOUBLE_EQ(sensor.initial_noise(), 4.903325e-2);
    EXPECT_TRUE(sensor.inverse_scaling_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(random_walk_sensor, stddev_variance_and_stiffness) {
    const RandomWalkSensor<3> sensor{SensorType::ACCELEROMETER, 100.0, 1.0e-2, 1.0e-3, 1.0e-1};
    EXPECT_DOUBLE_EQ(sensor.stddev(), 1.0e-2 * std::sqrt(100.0));
    EXPECT_DOUBLE_EQ(sensor.variance(), 1.0e-4 * 100.0);
    EXPECT_DOUBLE_EQ(sensor.variance(), sensor.stddev() * sensor.stddev());
    EXPECT_TRUE(sensor.stiffness().isApprox(Eigen::Matrix3d::Identity() / sensor.stddev()));
}

TEST(random_walk_sensor, setters) {
    RandomWalkSensor<3> sensor{SensorType::ACCELEROMETER, 100.0, 1.0e-2, 1.0e-3, 1.0e-1};
    sensor.set_frequency(400.0);
    EXPECT_DOUBLE_EQ(sensor.frequency(), 400.0);
    EXPECT_TRUE(sensor.stiffness().isApprox(Eigen::Matrix3d::Identity() / sensor.stddev()));
    sensor.set_noise_density(2.0e-2);
    EXPECT_DOUBLE_EQ(sensor.noise_density(), 2.0e-2);
    EXPECT_TRUE(sensor.stiffness().isApprox(Eigen::Matrix3d::Identity() / sensor.stddev()));
    sensor.set_bias_noise_density(2.0e-3);
    EXPECT_DOUBLE_EQ(sensor.bias_noise_density(), 2.0e-3);
    sensor.set_initial_noise(2.0e-1);
    EXPECT_DOUBLE_EQ(sensor.initial_noise(), 2.0e-1);
}

TEST(random_walk_sensor, set_properties) {
    RandomWalkSensor<3> sensor{SensorType::ACCELEROMETER, 100.0, 1.0e-2, 1.0e-3, 1.0e-1};
    sensor.set_properties(50.0, 3.0e-2);
    EXPECT_DOUBLE_EQ(sensor.frequency(), 50.0);
    EXPECT_DOUBLE_EQ(sensor.noise_density(), 3.0e-2);
    EXPECT_DOUBLE_EQ(sensor.bias_noise_density(), 1.0e-3);
    sensor.set_properties(25.0, 4.0e-2, 4.0e-3);
    EXPECT_DOUBLE_EQ(sensor.frequency(), 25.0);
    EXPECT_DOUBLE_EQ(sensor.noise_density(), 4.0e-2);
    EXPECT_DOUBLE_EQ(sensor.bias_noise_density(), 4.0e-3);
    EXPECT_TRUE(sensor.stiffness().isApprox(Eigen::Matrix3d::Identity() / sensor.stddev()));
}

TEST(random_walk_sensor, scalar_scaling) {
    nlohmann::json config = test_accelerometer_config();
    config["scaling"] = 2.0;
    const RandomWalkSensor<3> sensor{config};
    EXPECT_TRUE(sensor.inverse_scaling_matrix().isApprox(Eigen::Matrix3d::Identity() * 0.5));
    EXPECT_TRUE(sensor.scale_measurement(Eigen::Vector3d{2.0, 4.0, 6.0}).isApprox(Eigen::Vector3d{1.0, 2.0, 3.0}));
}

TEST(random_walk_sensor, per_axis_scaling) {
    nlohmann::json config = test_accelerometer_config();
    config["scaling"] = {1.0, 2.0, 4.0};
    const RandomWalkSensor<3> sensor{config};
    const Eigen::Matrix3d expected_inverse_scaling{Eigen::Vector3d{1.0, 0.5, 0.25}.asDiagonal()};
    EXPECT_TRUE(sensor.inverse_scaling_matrix().isApprox(expected_inverse_scaling));
    EXPECT_TRUE(sensor.scale_measurement(Eigen::Vector3d{1.0, 2.0, 4.0}).isApprox(Eigen::Vector3d::Ones()));
}

TEST(random_walk_sensor, per_axis_scaling_of_wrong_size_throws) {
    nlohmann::json config = test_accelerometer_config();
    config["scaling"] = {1.0, 2.0};
    EXPECT_THROW((RandomWalkSensor<3>{config}), std::runtime_error);
}

TEST(random_walk_sensor, set_scaling_matrix_stores_inverse) {
    RandomWalkSensor<2> sensor{SensorType::ACCELEROMETER, 100.0, 1.0e-2, 1.0e-3, 1.0e-1};
    const Eigen::Matrix2d scaling{Eigen::Vector2d{2.0, 4.0}.asDiagonal()};
    sensor.set_scaling_matrix(scaling);
    EXPECT_TRUE(sensor.inverse_scaling_matrix().isApprox(scaling.inverse()));
    EXPECT_TRUE((sensor.inverse_scaling_matrix() * scaling).isApprox(Eigen::Matrix2d::Identity()));
}

TEST(random_walk_sensor, invalid_config_throws) {
    nlohmann::json config = test_accelerometer_config();
    config.erase("frequency");
    EXPECT_THROW((RandomWalkSensor<3>{config}), std::exception);
    config = test_accelerometer_config();
    config["noise_density"] = -1.0;
    EXPECT_THROW((RandomWalkSensor<3>{config}), std::exception);
}

TEST(accelerometer, construction) {
    const Accelerometer<3> accelerometer{400.0, 5.5e-4, 4.9e-5, 4.9e-2};
    EXPECT_EQ(accelerometer.type(), SensorType::ACCELEROMETER);
    EXPECT_EQ(Accelerometer<2>::D, 2);
    EXPECT_EQ(Accelerometer<2>::DoF, 2);
    EXPECT_EQ(Accelerometer<3>::D, 3);
    EXPECT_EQ(Accelerometer<3>::DoF, 3);
    EXPECT_DOUBLE_EQ(accelerometer.frequency(), 400.0);
}

TEST(accelerometer, construction_from_config) {
    const Accelerometer<3> accelerometer{test_accelerometer_config()};
    EXPECT_EQ(accelerometer.type(), SensorType::ACCELEROMETER);
    EXPECT_DOUBLE_EQ(accelerometer.noise_density(), 5.5e-4);
}

TEST(accelerometer, wrong_sensor_type_throws) {
    EXPECT_THROW((Accelerometer<3>{test_gyroscope_config()}), std::exception);
}

TEST(gyroscope, construction) {
    const Gyroscope<3> gyroscope{200.0, 2.0e-5, 5.0e-6, 1.7e-3};
    EXPECT_EQ(gyroscope.type(), SensorType::GYROSCOPE);
    EXPECT_EQ(Gyroscope<2>::D, 2);
    EXPECT_EQ(Gyroscope<2>::DoF, 1);
    EXPECT_EQ(Gyroscope<3>::D, 3);
    EXPECT_EQ(Gyroscope<3>::DoF, 3);
    EXPECT_DOUBLE_EQ(gyroscope.frequency(), 200.0);
}

TEST(gyroscope, construction_from_config) {
    const Gyroscope<3> gyroscope{test_gyroscope_config()};
    EXPECT_EQ(gyroscope.type(), SensorType::GYROSCOPE);
    EXPECT_DOUBLE_EQ(gyroscope.noise_density(), 2.0e-5);
    EXPECT_DOUBLE_EQ(gyroscope.initial_noise(), 1.7e-3);
}

TEST(gyroscope, wrong_sensor_type_throws) {
    EXPECT_THROW((Gyroscope<3>{test_accelerometer_config()}), std::exception);
}

}
