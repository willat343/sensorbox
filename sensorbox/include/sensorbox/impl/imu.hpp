#ifndef SENSORBOX_IMPL_IMU_HPP
#define SENSORBOX_IMPL_IMU_HPP

#include "sensorbox/imu.hpp"

namespace sensorbox {

template<int D>
ImuMeasurement<D>::ImuMeasurement()
    : ImuMeasurement(Timestamp(Duration::zero()), std::string(), AngularVelocity::Zero(), LinearAcceleration::Zero()) {}

template<int D>
ImuMeasurement<D>::ImuMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const AngularVelocity& angular_velocity_, const LinearAcceleration& linear_acceleration_)
    : UnaryMeasurement(timestamp_, frame_),
      angular_velocity_(angular_velocity_),
      linear_acceleration_(linear_acceleration_) {}

template<int D>
inline auto ImuMeasurement<D>::angular_velocity() const -> const AngularVelocity& {
    return angular_velocity_;
}

template<int D>
inline auto ImuMeasurement<D>::angular_velocity() -> AngularVelocity& {
    return const_cast<AngularVelocity&>(std::as_const(*this).angular_velocity());
}

template<int D>
inline auto ImuMeasurement<D>::linear_acceleration() const -> const LinearAcceleration& {
    return linear_acceleration_;
}

template<int D>
inline auto ImuMeasurement<D>::linear_acceleration() -> LinearAcceleration& {
    return const_cast<LinearAcceleration&>(std::as_const(*this).linear_acceleration());
}

}

#endif
