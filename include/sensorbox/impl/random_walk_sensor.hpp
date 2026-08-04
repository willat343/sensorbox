#ifndef SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP
#define SENSORBOX_IMPL_RANDOM_WALK_SENSOR_HPP

#include <convert/convert.hpp>
#include <cppbox/exceptions.hpp>

#include "mathbox/stiffness.hpp"
#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

template<int DoF_>
inline RandomWalkSensor<DoF_>::RandomWalkSensor(const SensorType type, const double frequency_,
        const double noise_density_, const double bias_noise_density_, const double initial_noise_)
    : Sensor(type), inverse_scaling_matrix_(ScalingMatrix::Identity()) {
    set_properties(frequency_, noise_density_, bias_noise_density_);
    initial_noise__ = initial_noise_;
}

template<int DoF_>
inline RandomWalkSensor<DoF_>::RandomWalkSensor(const nlohmann::json& config, const bool validate)
    : Sensor(config, false), JsonLoadable<RandomWalkSensorSchemaFilepath, sensorbox_schema_loader>(config, validate) {
    set_properties(config["frequency"].get<double>(), config["noise_density"].get<double>(),
            config["bias_noise_density"].get<double>());
    initial_noise__ = config["initial_noise"].get<double>();

    // Parse optional axis scaling matrix S from "scaling", either a scalar (uniform scaling) or a per-axis vector
    // (the diagonal of S), defaulting to the identity matrix if not present.
    if (config.contains("scaling")) {
        const nlohmann::json& scaling = config["scaling"];
        if (scaling.is_array()) {
            throw_if(scaling.size() != DoF, "RandomWalkSensor: Expected size of 'scaling' vector in json was " +
                                                    std::to_string(DoF) + " but was " + std::to_string(scaling.size()) +
                                                    ".");
            set_scaling_matrix(convert::to<Eigen::Vector<double, DoF>>(
                    scaling.template get<std::array<double, std::size_t(DoF)>>())
                            .asDiagonal());
        } else {
            set_scaling_matrix(ScalingMatrix::Identity() * scaling.get<double>());
        }
    } else {
        set_scaling_matrix(ScalingMatrix::Identity());
    }
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
inline auto RandomWalkSensor<DoF_>::inverse_scaling_matrix() const -> const ScalingMatrix& {
    return inverse_scaling_matrix_;
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
inline auto RandomWalkSensor<DoF_>::scale_measurement(const Eigen::Vector<double, DoF>& measurement) const
        -> Eigen::Vector<double, DoF> {
    return inverse_scaling_matrix_ * measurement;
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
inline void RandomWalkSensor<DoF_>::set_scaling_matrix(const ScalingMatrix& scaling_matrix_) {
    inverse_scaling_matrix_ = scaling_matrix_.inverse();
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::stddev() const {
    return noise_density() * std::sqrt(frequency());
}

template<int DoF_>
inline auto RandomWalkSensor<DoF_>::stiffness() const -> const Stiffness& {
    return stiffness_;
}

template<int DoF_>
inline void RandomWalkSensor<DoF_>::update_stiffness() {
    stiffness_ = math::stiffness_from_sigma<DoF>(stddev());
}

template<int DoF_>
inline double RandomWalkSensor<DoF_>::variance() const {
    return noise_density() * noise_density() * frequency();
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
