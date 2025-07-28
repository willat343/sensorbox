#ifndef SENSORBOX_SENSOR_HPP
#define SENSORBOX_SENSOR_HPP

#include <cppbox/enum.hpp>
#include <nlohmann/json.hpp>

#include "sensorbox/measurement.hpp"

namespace sensorbox {

CREATE_SMART_ENUM(SensorTypeBase, ACCELEROMETER, DIRECT_POSE, GYROSCOPE, IMU)

class SensorType : public SensorTypeBase {
public:
    SensorType()
        : SensorTypeBase() {}

    SensorType(const SensorTypeBase::Identifiers identifier)
        : SensorTypeBase(identifier) {}

    SensorType(const std::string& string)
        : SensorTypeBase(string) {}

    inline MeasurementType measurement_type() const {
        switch ((*this)()) {
            case ACCELEROMETER:
                return MeasurementType::LINEAR_ACCELERATION;
            case DIRECT_POSE:
                return MeasurementType::POSE;
            case GYROSCOPE:
                return MeasurementType::ANGULAR_VELOCITY;
            case IMU:
                return MeasurementType::IMU_MEASUREMENT;
            default:
                throw std::runtime_error("No MeasurementType for SensorType " + std::string(*this) + ".");
        }
    }
};

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

CREATE_SMART_ENUM_FREE_FUNCTIONS(sensorbox::SensorTypeBase)

#endif
