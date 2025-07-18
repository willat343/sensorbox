#ifndef SENSORBOX_IMU_HPP
#define SENSORBOX_IMU_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <string>

#include "sensorbox/accelerometer.hpp"
#include "sensorbox/gyroscope.hpp"
#include "sensorbox/sensor.hpp"
#include "sensorbox/unary.hpp"

namespace sensorbox {

template<int D_>
class Imu : public Sensor {
public:
    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Construct an instance of the class from a json config with Sensor structure plus:
     * ```json
     * "accelerometer": <Accelerometer>,
     * "gyroscope": <Gyroscope>
     * ```
     *
     * @param config
     */
    explicit Imu(const nlohmann::json& config);

    const Accelerometer<D>& accelerometer() const;

    const Gyroscope<D>& gyroscope() const;

private:
    Accelerometer<D> accelerometer_;
    Gyroscope<D> gyroscope_;
};

template<int D_>
class ImuMeasurement : public UnaryMeasurement {
public:
    using Clock = UnaryMeasurement::Clock;
    using Duration = UnaryMeasurement::Duration;
    using Timestamp = UnaryMeasurement::Timestamp;
    static constexpr int D = D_;
    static constexpr int AccelDoF = D;
    static constexpr int GyroDoF = D * (D - 1) / 2;
    using AngularVelocity = Eigen::Matrix<double, GyroDoF, 1>;
    using LinearAcceleration = Eigen::Matrix<double, AccelDoF, 1>;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;

    explicit ImuMeasurement();

    /**
     * @brief Construct a Imu Measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. imu_frame
     * @param angular_velocity_ angular velocity (rad/s)
     * @param linear_acceleration_ linear acceleration (m/s^2)
     */
    explicit ImuMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const AngularVelocity& angular_velocity_, const LinearAcceleration& linear_acceleration_);

    const AngularVelocity& angular_velocity() const;

    AngularVelocity& angular_velocity();

    const LinearAcceleration& linear_acceleration() const;

    LinearAcceleration& linear_acceleration();

    ImuMeasurement transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const;

private:
    AngularVelocity angular_velocity_;
    LinearAcceleration linear_acceleration_;
};

}

#include "sensorbox/impl/imu.hpp"

#endif
