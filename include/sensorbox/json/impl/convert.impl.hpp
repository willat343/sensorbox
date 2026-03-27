#ifndef SENSORBOX_JSON_IMPL_CONVERT_IMPL_HPP
#define SENSORBOX_JSON_IMPL_CONVERT_IMPL_HPP

#include <cppbox/time.hpp>

#include "sensorbox/impl/sensorbox.hpp"
#include "sensorbox/json/impl/convert.hpp"

namespace sensorbox {

SENSORBOX_INLINE void from_json(const nlohmann::json& j, SensorType& type) {
    type = SensorType(j.get<std::string>());
}

SENSORBOX_INLINE void to_json(nlohmann::json& j, const ContactClassifications& contact_classifications) {
    j = nlohmann::json::object();
    j["timestamp"] = cppbox::to_nsec(contact_classifications.timestamp());
    j["values"] = nlohmann::json::array();
    for (const auto& [key, classification] : contact_classifications.classifications()) {
        j["values"].push_back({{"key", key}, {"value", classification}});
    }
}

}

#endif
