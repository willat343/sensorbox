#ifndef SENSORBOX_IMPL_IMU_HPP
#define SENSORBOX_IMPL_IMU_HPP

#include <cppbox/exceptions.hpp>

#include "sensorbox/imu.hpp"

namespace sensorbox {

template<int D_>
Imu<D_>::Imu(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<ImuSchemaFilepath, sensorbox_schema_loader>(config, validate),
      accelerometer_(config.at("accelerometer"), false),
      gyroscope_(config.at("gyroscope"), false) {
    assert(this->type() == SensorType::IMU);
}

template<int D_>
inline auto Imu<D_>::accelerometer() const -> const Accelerometer<D>& {
    return accelerometer_;
}

template<int D_>
inline auto Imu<D_>::gyroscope() const -> const Gyroscope<D>& {
    return gyroscope_;
}

template<int D_>
ImuMeasurement<D_>::ImuMeasurement()
    : ImuMeasurement(Timestamp(Duration::zero()), std::string(), AngularVelocity::Zero(), LinearAcceleration::Zero()) {}

template<int D_>
ImuMeasurement<D_>::ImuMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const AngularVelocity& angular_velocity_, const LinearAcceleration& linear_acceleration_)
    : UnaryMeasurement(timestamp_, frame_),
      angular_velocity_(angular_velocity_),
      linear_acceleration_(linear_acceleration_) {}

template<int D_>
inline auto ImuMeasurement<D_>::angular_velocity() const -> const AngularVelocity& {
    return angular_velocity_;
}

template<int D_>
inline auto ImuMeasurement<D_>::angular_velocity() -> AngularVelocity& {
    return const_cast<AngularVelocity&>(std::as_const(*this).angular_velocity());
}

template<int D_>
inline auto ImuMeasurement<D_>::linear_acceleration() const -> const LinearAcceleration& {
    return linear_acceleration_;
}

template<int D_>
inline auto ImuMeasurement<D_>::linear_acceleration() -> LinearAcceleration& {
    return const_cast<LinearAcceleration&>(std::as_const(*this).linear_acceleration());
}

template<int D_>
inline auto ImuMeasurement<D_>::transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const
        -> ImuMeasurement {
    throw_if(T_N_F.isApprox(Pose::Identity()), "Transformation of IMU measurements (with non-identity matrix) not yet "
                                               "implemented. Must be done very carefully.");
    return ImuMeasurement(timestamp(), new_frame, angular_velocity(), linear_acceleration());
}

}

#endif
