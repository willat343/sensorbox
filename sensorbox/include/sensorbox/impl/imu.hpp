#ifndef SENSORBOX_IMPL_IMU_HPP
#define SENSORBOX_IMPL_IMU_HPP

#include "sensorbox/imu.hpp"

namespace sensorbox {

inline Eigen::Vector3d Imu::rpy() const {
    return orientation.toRotationMatrix().eulerAngles(0, 1, 2);
}

}

#endif
