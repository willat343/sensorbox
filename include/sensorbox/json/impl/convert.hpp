#ifndef SENSORBOX_JSON_IMPL_CONVERT_HPP
#define SENSORBOX_JSON_IMPL_CONVERT_HPP

#include <cppbox/time.hpp>
#include <optional>

#include "sensorbox/json/convert.hpp"

namespace sensorbox {

template<int D>
void to_json(nlohmann::json& j, const WrenchMeasurements<D>& wrench_measurements) {
    j = nlohmann::json::object();
    j["timestamp"] = cppbox::to_nsec(wrench_measurements.timestamp());
    j["wrenches"] = nlohmann::json::array();
    for (const WrenchMeasurement<D>& wrench_measurement : wrench_measurements.measurements()) {
        nlohmann::json wrench_json{{"frame_id", wrench_measurement.frame()},
                {"child_frame_id", wrench_measurement.child_frame()}};
        if (const std::optional<typename WrenchMeasurement<D>::Force>& force = wrench_measurement.force()) {
            wrench_json["force"] = {{"x", (*force)[0]}, {"y", (*force)[1]}};
            if constexpr (D == 3) {
                wrench_json["force"]["z"] = (*force)[2];
            }
        }
        if (const std::optional<typename WrenchMeasurement<D>::Torque>& torque = wrench_measurement.torque()) {
            if constexpr (D == 3) {
                wrench_json["torque"] = {{"x", (*torque)[0]}, {"y", (*torque)[1]}, {"z", (*torque)[2]}};
            } else {
                wrench_json["torque"] = (*torque)[0];
            }
        }
        j["wrenches"].push_back(wrench_json);
    }
}

}

#if SENSORBOX_HEADER_ONLY
#include "sensorbox/json/impl/convert.impl.hpp"
#endif

#endif
