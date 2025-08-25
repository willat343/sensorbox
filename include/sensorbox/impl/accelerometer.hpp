#ifndef SENSORBOX_IMPL_ACCELEROMETER_HPP
#define SENSORBOX_IMPL_ACCELEROMETER_HPP

#include "sensorbox/accelerometer.hpp"

namespace sensorbox {

template<int D_>
inline Accelerometer<D_>::Accelerometer(const nlohmann::json& config, const bool validate)
    : RandomWalkSensor<D_>(config, false),
      JsonLoadable<AccelerometerSchemaFilepath, sensorbox_schema_loader>(config, validate) {
    assert(this->type() == SensorType::ACCELEROMETER);
}

template<int D_>
inline Accelerometer<D_>::Accelerometer(const double frequency_, const double noise_density_,
        const double bias_noise_density_, const double initial_noise_)
    : RandomWalkSensor<D_>(SensorType::ACCELEROMETER, frequency_, noise_density_, bias_noise_density_, initial_noise_) {
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class Accelerometer<2>;
extern template class Accelerometer<3>;
extern template class JsonLoadable<AccelerometerSchemaFilepath, sensorbox_schema_loader>;

}
#endif

#endif
