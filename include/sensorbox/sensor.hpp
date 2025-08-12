#ifndef SENSORBOX_SENSOR_HPP
#define SENSORBOX_SENSOR_HPP

#include <cppbox/enum.hpp>
#include <nlohmann/json.hpp>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/measurement.hpp"

#ifndef SENSORBOX_SCHEMAS_DIRECTORY
#error "SENSORBOX_SCHEMAS_DIRECTORY is not defined."
#endif

namespace sensorbox {

constexpr SchemaFilepath AccelerometerSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Accelerometer.schema.json"};
constexpr SchemaFilepath DirectPoseSensorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "DirectPoseSensor.schema.json"};
constexpr SchemaFilepath GyroscopeSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Gyroscope.schema.json"};
constexpr SchemaFilepath ImuSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Imu.schema.json"};
constexpr SchemaFilepath RandomWalkSensorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "RandomWalkSensor.schema.json"};
constexpr SchemaFilepath SensorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Sensor.schema.json"};

CREATE_SMART_ENUM(SensorTypeBase, ACCELEROMETER, CONTACTS_CLASSIFIER, DIRECT_POSE, GYROSCOPE, IMU)

class SensorType : public SensorTypeBase {
public:
    SensorType() : SensorTypeBase() {}

    SensorType(const SensorTypeBase::Identifiers identifier) : SensorTypeBase(identifier) {}

    SensorType(const std::string& string) : SensorTypeBase(string) {}

    inline MeasurementType measurement_type() const {
        switch ((*this)()) {
            case ACCELEROMETER:
                return MeasurementType::LINEAR_ACCELERATION;
            case CONTACTS_CLASSIFIER:
                return MeasurementType::CONTACT_CLASSIFICATIONS;
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

class Sensor : public JsonLoadable<SensorSchemaFilepath, sensorbox_schema_loader> {
public:
    explicit Sensor(const SensorType type_);

    /**
     * @brief Construct an instance of the class from a json according to schema.
     *
     * @param config
     * @param validate
     */
    explicit Sensor(const nlohmann::json& config, const bool validate = true);

    SensorType type() const;

    const std::string& make() const;

    const std::string& model() const;

private:
    SensorType type_;
    std::string make_;
    std::string model_;
};

}

#endif
