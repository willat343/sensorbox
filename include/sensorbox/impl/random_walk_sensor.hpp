#ifndef SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP
#define SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP

#include "mathbox/stiffness.hpp"
#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

template<int DoF_>
inline RandomWalkSensor<DoF_>::RandomWalkSensor(const SensorType type, const double frequency_, const double noise_density_,
        const double bias_noise_density_, const double initial_noise_)
    : Sensor(type) {
    set_properties(frequency_, noise_density_, bias_noise_density_);
    initial_noise__ = initial_noise_;
}

template<int DoF_>
inline RandomWalkSensor<DoF_>::RandomWalkSensor(const nlohmann::json& config, const bool validate)
    : Sensor(config, false), JsonLoadable<RandomWalkSensorSchemaFilepath, sensorbox_schema_loader>(config, validate) {
    set_properties(config["frequency"].get<double>(), config["noise_density"].get<double>(),
            config["bias_noise_density"].get<double>());
    initial_noise__ = config["initial_noise"].get<double>();
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::bias_noise_density() const {
    return bias_noise_density__;
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::frequency() const {
    return frequency__;
}
template<int DoF_>
inline double RandomWalkSensor<DoF_>::initial_noise() const {
    return initial_noise__;
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::noise_density() const {
    return noise_density__;
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::period() const {
    return 1.0 / frequency();
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_bias_noise_density(const double bias_noise_density_) {
    bias_noise_density__ = bias_noise_density_;
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_frequency(const double frequency_) {
    frequency__ = frequency_;
    update_stiffness();
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_initial_noise(const double initial_noise_) {
    initial_noise__ = initial_noise_;
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_noise_density(const double noise_density_) {
    noise_density__ = noise_density_;
    update_stiffness();
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_properties(const double frequency_, const double noise_density_) {
    frequency__ = frequency_;
    noise_density__ = noise_density_;
    update_stiffness();
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::set_properties(const double frequency_, const double noise_density_,
        const double bias_noise_density_) {
    set_properties(frequency_, noise_density_);
    set_bias_noise_density(bias_noise_density_);
}

template<int DoF_>
inline auto RandomWalkSensor<DoF_>::stiffness() const -> const Stiffness& {
    return stiffness_;
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::update_stiffness() {
    stiffness_ = math::stiffness_from_sigma<DoF>(noise_density() * std::sqrt(frequency()));
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class RandomWalkSensor<1>;
extern template class RandomWalkSensor<2>;
extern template class RandomWalkSensor<3>;
extern template class JsonLoadable<RandomWalkSensorSchemaFilepath, sensorbox_schema_loader>;

}
#endif

#endif
