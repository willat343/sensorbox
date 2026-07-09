#ifndef SENSORBOX_SENSOR_HPP
#define SENSORBOX_SENSOR_HPP

#include <cppbox/enum.hpp>
#include <cppbox/exceptions.hpp>
#include <nlohmann/json.hpp>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/measurement.hpp"

#ifndef SENSORBOX_SCHEMAS_DIRECTORY
#error "SENSORBOX_SCHEMAS_DIRECTORY is not defined."
#endif

namespace sensorbox {

constexpr inline SchemaFilepath AccelerometerSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Accelerometer.schema.json"};
constexpr inline SchemaFilepath ActuatorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Actuator.schema.json"};
constexpr inline SchemaFilepath CurrentSensorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "CurrentSensor.schema.json"};
constexpr inline SchemaFilepath DirectPoseSensorSchemaFilepath{
        SENSORBOX_SCHEMAS_DIRECTORY "DirectPoseSensor.schema.json"};
constexpr inline SchemaFilepath DirectPositionSensorSchemaFilepath{
        SENSORBOX_SCHEMAS_DIRECTORY "DirectPositionSensor.schema.json"};
constexpr inline SchemaFilepath EncoderSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Encoder.schema.json"};
constexpr inline SchemaFilepath GyroscopeSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Gyroscope.schema.json"};
constexpr inline SchemaFilepath ImuSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Imu.schema.json"};
constexpr inline SchemaFilepath RandomWalkSensorSchemaFilepath{
        SENSORBOX_SCHEMAS_DIRECTORY "RandomWalkSensor.schema.json"};
constexpr inline SchemaFilepath SensorSchemaFilepath{SENSORBOX_SCHEMAS_DIRECTORY "Sensor.schema.json"};

CREATE_SMART_ENUM(SensorTypeBase, ACCELEROMETER, ACTUATOR, ACTUATORS, CONTACTS_CLASSIFIER, CURRENT, DIRECT_POSE,
        DIRECT_POSITION, ENCODER, GYROSCOPE, IMU)

/**
 * @brief Sensor type, each of which is associated with a measurement type.
 *
 */
class SensorType : public SensorTypeBase {
public:
    SensorType() : SensorTypeBase() {}

    SensorType(const SensorTypeBase::Identifiers identifier) : SensorTypeBase(identifier) {}

    SensorType(const std::string& string) : SensorTypeBase(string) {}

    inline MeasurementType measurement_type() const {
        switch ((*this)()) {
            case ACCELEROMETER:
                return MeasurementType::LINEAR_ACCELERATION;
            case ACTUATOR:
                return MeasurementType::ACTUATOR_MEASUREMENT;
            case ACTUATORS:
                return MeasurementType::ACTUATOR_MEASUREMENTS;
            case CONTACTS_CLASSIFIER:
                return MeasurementType::CONTACT_CLASSIFICATIONS;
            case CURRENT:
                return MeasurementType::CURRENT;
            case DIRECT_POSE:
                return MeasurementType::POSE;
            case DIRECT_POSITION:
                return MeasurementType::POSITION;
            case ENCODER:
                return MeasurementType::ANGLE;
            case GYROSCOPE:
                return MeasurementType::ANGULAR_VELOCITY;
            case IMU:
                return MeasurementType::IMU_MEASUREMENT;
            default:
                throw_here("No MeasurementType for SensorType " + std::string(*this) + ".");
        }
    }
};

/**
 * @brief Sensor class
 *
 */
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
    /**
     * @brief Sensor type
     *
     */
    SensorType type_;

    /**
     * @brief Make/Company of sensor
     *
     */
    std::string make_;

    /**
     * @brief Sensor model name
     *
     */
    std::string model_;
};

}

#include "sensorbox/impl/sensor.hpp"

#endif
