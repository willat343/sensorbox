#ifndef SENSORBOX_JSON_CONVERT_HPP
#define SENSORBOX_JSON_CONVERT_HPP

#include <nlohmann/json.hpp>

#include "sensorbox/contact.hpp"
#include "sensorbox/sensor.hpp"
#include "sensorbox/wrench.hpp"

namespace sensorbox {

void from_json(const nlohmann::json& j, SensorType& type);

void to_json(nlohmann::json& j, const ContactClassifications& contact_classifications);

/**
 * @brief Convert wrench measurements to JSON, following the TemporalWrenches2D or TemporalWrenches3D schema: the
 * timestamp in nanoseconds and, for each wrench, its frame, its child frame and whichever of its force and torque are
 * present, where the torque is a scalar in 2D.
 *
 * @tparam D dimension
 * @param j JSON
 * @param wrench_measurements wrench measurements
 */
template<int D>
void to_json(nlohmann::json& j, const WrenchMeasurements<D>& wrench_measurements);

}

#include "sensorbox/json/impl/convert.hpp"

#endif
