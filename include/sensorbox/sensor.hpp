#ifndef SENSORBOX_SENSOR_HPP
#define SENSORBOX_SENSOR_HPP

#include <cppbox/enum.hpp>
#include <nlohmann/json.hpp>

namespace sensorbox {

CREATE_SMART_ENUM(SensorType, ACCELEROMETER, GYROSCOPE, IMU)

class Sensor {
public:
    explicit Sensor(const SensorType type_);

    /**
     * @brief Construct an instance of the class from a json config with structure:
     * ```json
     * "type": <string>,
     * "make": <string>,
     * "model": <string>
     * ```
     *
     * @param config
     */
    explicit Sensor(const nlohmann::json& config);

    SensorType type() const;

    const std::string& make() const;

    const std::string& model() const;

private:
    SensorType type_;
    std::string make_;
    std::string model_;
};

}

CREATE_SMART_ENUM_FREE_FUNCTIONS(sensorbox::SensorType)

#endif
