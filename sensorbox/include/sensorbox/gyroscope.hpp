#ifndef SENSORBOX_GYROSCOPE_HPP
#define SENSORBOX_GYROSCOPE_HPP

#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

/**
 * @brief Gyroscope properties. Sensor units are rad * s^{-1}.
 *
 * @tparam D dimension (e.g. 3 for 3D)
 */
template<int D>
using GyroscopeProperties = RandomWalkSensorProperties<D*(D - 1) / 2>;

}

#endif
