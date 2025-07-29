#include "sensorbox/sensor.hpp"

namespace sensorbox {

Sensor::Sensor(const SensorType type_) : type_(type_), make_(""), model_("") {}

Sensor::Sensor(const nlohmann::json& config)
    : type_(SensorType{config["type"].get<std::string>()}),
      make_(config.contains("make") ? config["make"].get<std::string>() : std::string()),
      model_(config.contains("model") ? config["model"].get<std::string>() : std::string()) {}

SensorType Sensor::type() const {
    return type_;
}

const std::string& Sensor::make() const {
    return make_;
}

const std::string& Sensor::model() const {
    return model_;
}

}
