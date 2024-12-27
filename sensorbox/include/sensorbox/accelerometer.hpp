#ifndef SENSORBOX_ACCELEROMETER_HPP
#define SENSORBOX_ACCELEROMETER_HPP

#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

/**
 * @brief Accelerometer properties. Sensor units are m * s^{-2}.
 *
 * @tparam D dimension (e.g. 3 for 3D)
 */
template<int D>
using AccelerometerProperties = RandomWalkSensorProperties<D>;

}

#endif
