#ifndef SENSORBOX_MEASUREMENT_HPP
#define SENSORBOX_MEASUREMENT_HPP

#include <cppbox/enum.hpp>

namespace sensorbox {

CREATE_SMART_ENUM(MeasurementType, ANGULAR_VELOCITY, IMU_MEASUREMENT, LINEAR_ACCELERATION, POSE)

}

#endif
