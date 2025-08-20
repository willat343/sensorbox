#ifndef SENSORBOX_CURRENT_HPP
#define SENSORBOX_CURRENT_HPP

#include <Eigen/Core>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

class CurrentSensor : public Sensor, public JsonLoadable<CurrentSensorSchemaFilepath, sensorbox_schema_loader> {
public:
    using Stiffness = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Construct an instance of the class from a json config with CurrentSensor structure.
     *
     * @param config
     * @param validate
     */
    explicit CurrentSensor(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get stiffness for sensor.
     *
     * @return const Stiffness&
     */
    const Stiffness stiffness() const;

private:
    /**
     * @brief Stiffness in A^{-1} of the sensor's current readings
     *
     */
    Stiffness stiffness_;
};

}

#include "sensorbox/impl/current.hpp"

#endif
