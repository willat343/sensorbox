#ifndef SENSORBOX_IMPL_CURRENT_HPP
#define SENSORBOX_IMPL_CURRENT_HPP

#include "sensorbox/current.hpp"
#include "sensorbox/stiffness.hpp"

namespace sensorbox {

inline CurrentSensor::CurrentSensor(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<CurrentSensorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      stiffness_(stiffness_from_config<1>(config)) {}

inline auto CurrentSensor::stiffness() const -> const Stiffness {
    return stiffness_;
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class JsonLoadable<CurrentSensorSchemaFilepath, sensorbox_schema_loader>;

}
#endif

#endif
