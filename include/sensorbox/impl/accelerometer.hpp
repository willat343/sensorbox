#ifndef SENSORBOX_IMPL_ACCELEROMETER_HPP
#define SENSORBOX_IMPL_ACCELEROMETER_HPP

#include "sensorbox/accelerometer.hpp"

namespace sensorbox {

template<int D_>
Accelerometer<D_>::Accelerometer(const nlohmann::json& config)
    : RandomWalkSensor<D_>(config), JsonLoadable<AccelerometerSchemaFilepath, sensorbox_schema_loader>(config) {
    assert(this->type() == SensorType::ACCELEROMETER);
}

template<int D_>
Accelerometer<D_>::Accelerometer(const double frequency_, const double noise_density_, const double bias_noise_density_,
        const double initial_noise_)
    : RandomWalkSensor<D_>(SensorType::ACCELEROMETER, frequency_, noise_density_, bias_noise_density_, initial_noise_) {
}

}

#endif
