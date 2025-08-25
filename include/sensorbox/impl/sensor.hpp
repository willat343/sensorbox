#ifndef SENSORBOX_IMPL_SENSOR_HPP
#define SENSORBOX_IMPL_SENSOR_HPP

#include "sensorbox/sensor.hpp"

namespace sensorbox {

inline Sensor::Sensor(const SensorType type_) : type_(type_), make_(""), model_("") {}

inline Sensor::Sensor(const nlohmann::json& config, const bool validate)
    : JsonLoadable<SensorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      type_(SensorType{config["type"].get<std::string>()}),
      make_(config.contains("make") ? config["make"].get<std::string>() : std::string()),
      model_(config.contains("model") ? config["model"].get<std::string>() : std::string()) {}

inline SensorType Sensor::type() const {
    return type_;
}

inline const std::string& Sensor::make() const {
    return make_;
}

inline const std::string& Sensor::model() const {
    return model_;
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class JsonLoadable<SensorSchemaFilepath, sensorbox_schema_loader>;

}
#endif

#endif
