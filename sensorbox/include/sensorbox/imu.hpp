#ifndef SENSORBOX_IMU_HPP
#define SENSORBOX_IMU_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <string>

#include "sensorbox/unary.hpp"

namespace sensorbox {

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

private:
    AngularVelocity angular_velocity_;
    LinearAcceleration linear_acceleration_;
};

}

#include "sensorbox/impl/imu.hpp"

#endif
