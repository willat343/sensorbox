#ifndef SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP
#define SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP

#include "mathbox/stiffness.hpp"
#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

template<int DoF>
RandomWalkSensorProperties<DoF>::RandomWalkSensorProperties(const double frequency_, const double noise_density_,
        const double bias_noise_density_) {
    set_properties(frequency_, noise_density_, bias_noise_density_);
}

template<int DoF>
inline double RandomWalkSensorProperties<DoF>::bias_noise_density() const {
    return bias_noise_density__;
}

template<int DoF>
inline double RandomWalkSensorProperties<DoF>::frequency() const {
    return frequency__;
}

template<int DoF>
inline double RandomWalkSensorProperties<DoF>::noise_density() const {
    return noise_density__;
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::set_bias_noise_density(const double bias_noise_density_) {
    bias_noise_density__ = bias_noise_density_;
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::set_frequency(const double frequency_) {
    frequency__ = frequency_;
    update_stiffness();
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::set_noise_density(const double noise_density_) {
    noise_density__ = noise_density_;
    update_stiffness();
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::set_properties(const double frequency_, const double noise_density_) {
    frequency__ = frequency_;
    noise_density__ = noise_density_;
    update_stiffness();
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::set_properties(const double frequency_, const double noise_density_,
        const double bias_noise_density_) {
    set_properties(frequency_, noise_density_);
    set_bias_noise_density(bias_noise_density_);
}

template<int DoF>
inline auto RandomWalkSensorProperties<DoF>::stiffness() const -> const Stiffness& {
    return stiffness_;
}

template<int DoF>
inline void RandomWalkSensorProperties<DoF>::update_stiffness() {
    stiffness_ = math::stiffness_from_sigma<DoF>(noise_density() * std::sqrt(frequency()));
}

}

#endif
