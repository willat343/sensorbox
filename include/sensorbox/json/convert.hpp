#ifndef SENSORBOX_JSON_CONVERT_HPP
#define SENSORBOX_JSON_CONVERT_HPP

#include <nlohmann/json.hpp>

#include "sensorbox/contact.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

void from_json(const nlohmann::json& j, SensorType& type);

void to_json(nlohmann::json& j, const ContactClassifications& contact_classifications);

}

#endif
