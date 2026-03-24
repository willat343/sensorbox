#ifndef SENSORBOX_JSON_IMPL_CONVERT_IMPL_HPP
#define SENSORBOX_JSON_IMPL_CONVERT_IMPL_HPP

#include "sensorbox/impl/sensorbox.hpp"
#include "sensorbox/json/impl/convert.hpp"

namespace sensorbox {

SENSORBOX_INLINE void from_json(const nlohmann::json& j, SensorType& type) {
    type = SensorType(j.get<std::string>());
}

}

#endif
