#ifndef SENSORBOX_IMPL_GYROSCOPE_HPP
#define SENSORBOX_IMPL_GYROSCOPE_HPP

#include "sensorbox/gyroscope.hpp"

namespace sensorbox {

template<int D_>
Gyroscope<D_>::Gyroscope(const nlohmann::json& config) : RandomWalkSensor<D_>(config) {}

template<int D_>
Gyroscope<D_>::Gyroscope(const double frequency_, const double noise_density_, const double bias_noise_density_,
        const double initial_noise_)
    : RandomWalkSensor<D_>(SensorType::GYROSCOPE, frequency_, noise_density_, bias_noise_density_, initial_noise_) {}

}

#endif
