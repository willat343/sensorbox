#include "sensorbox/sensor.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace sensorbox {

TEST(schema_filepath, string_excludes_null_terminator) {
    static constexpr SchemaFilepath filepath{"MyClass.schema.json"};
    static_assert(filepath.string() == std::string_view{"MyClass.schema.json"});
    EXPECT_EQ(filepath.string().size(), std::string_view{"MyClass.schema.json"}.size());
    EXPECT_EQ(std::string(filepath.string()), std::string("MyClass.schema.json"));
}

TEST(schema_filepath, schemas_exist) {
    for (const std::string_view schema_filepath : {SensorSchemaFilepath.string(), AccelerometerSchemaFilepath.string(),
                 ActuatorSchemaFilepath.string(), CurrentSensorSchemaFilepath.string(),
                 DirectPoseSensorSchemaFilepath.string(), DirectPositionSensorSchemaFilepath.string(),
                 EncoderSchemaFilepath.string(), GyroscopeSchemaFilepath.string(), ImuSchemaFilepath.string(),
                 RandomWalkSensorSchemaFilepath.string()}) {
        EXPECT_TRUE(std::filesystem::exists(std::filesystem::path{schema_filepath})) << schema_filepath;
    }
}

TEST(sensor_type, measurement_type) {
    EXPECT_EQ(SensorType(SensorType::ACCELEROMETER).measurement_type(), MeasurementType::LINEAR_ACCELERATION);
    EXPECT_EQ(SensorType(SensorType::ACTUATOR).measurement_type(), MeasurementType::ACTUATOR_MEASUREMENT);
    EXPECT_EQ(SensorType(SensorType::ACTUATORS).measurement_type(), MeasurementType::ACTUATOR_MEASUREMENTS);
    EXPECT_EQ(SensorType(SensorType::CONTACTS_CLASSIFIER).measurement_type(), MeasurementType::CONTACT_CLASSIFICATIONS);
    EXPECT_EQ(SensorType(SensorType::CURRENT).measurement_type(), MeasurementType::CURRENT);
    EXPECT_EQ(SensorType(SensorType::DIRECT_POSE).measurement_type(), MeasurementType::POSE);
    EXPECT_EQ(SensorType(SensorType::DIRECT_POSITION).measurement_type(), MeasurementType::POSITION);
    EXPECT_EQ(SensorType(SensorType::ENCODER).measurement_type(), MeasurementType::ANGLE);
    EXPECT_EQ(SensorType(SensorType::GYROSCOPE).measurement_type(), MeasurementType::ANGULAR_VELOCITY);
    EXPECT_EQ(SensorType(SensorType::IMU).measurement_type(), MeasurementType::IMU_MEASUREMENT);
}

TEST(sensor_type, every_sensor_type_has_a_measurement_type) {
    for (const SensorTypeBase::Identifiers identifier : SensorTypeBase::identifiers) {
        EXPECT_NO_THROW(SensorType(identifier).measurement_type()) << std::string(SensorType(identifier));
    }
}

TEST(sensor_type, string_conversion) {
    EXPECT_EQ(SensorType(std::string("IMU")), SensorType::IMU);
    EXPECT_EQ(std::string(SensorType(SensorType::IMU)), std::string("IMU"));
    EXPECT_THROW(SensorType(std::string("NOT_A_SENSOR_TYPE")), std::runtime_error);
}

TEST(sensor, construction_from_type) {
    const Sensor sensor{SensorType{SensorType::ENCODER}};
    EXPECT_EQ(sensor.type(), SensorType::ENCODER);
    EXPECT_TRUE(sensor.make().empty());
    EXPECT_TRUE(sensor.model().empty());
}

TEST(sensor, construction_from_config) {
    const nlohmann::json config = {{"type", "GYROSCOPE"}, {"make", "EPSON"}, {"model", "G365PDF1"}};
    const Sensor sensor{config};
    EXPECT_EQ(sensor.type(), SensorType::GYROSCOPE);
    EXPECT_EQ(sensor.make(), "EPSON");
    EXPECT_EQ(sensor.model(), "G365PDF1");
}

TEST(sensor, construction_from_config_without_optional_fields) {
    const Sensor sensor{nlohmann::json{{"type", "IMU"}}};
    EXPECT_EQ(sensor.type(), SensorType::IMU);
    EXPECT_TRUE(sensor.make().empty());
    EXPECT_TRUE(sensor.model().empty());
}

TEST(sensor, invalid_config_throws) {
    // Missing required "type" field
    EXPECT_THROW(Sensor(nlohmann::json{{"make", "EPSON"}}), std::exception);
    // "type" is not one of the known sensor types
    EXPECT_THROW(Sensor(nlohmann::json{{"type", "NOT_A_SENSOR_TYPE"}}), std::exception);
}

TEST(sensor, validation_can_be_skipped) {
    // An invalid config is accepted when validate is false, provided the fields required for construction exist
    EXPECT_NO_THROW(Sensor(nlohmann::json{{"type", "IMU"}, {"unexpected_field", 1.0}}, false));
}

TEST(json_loadable, validate) {
    using SensorJsonLoadable = JsonLoadable<SensorSchemaFilepath, sensorbox_schema_loader>;
    EXPECT_NO_THROW(SensorJsonLoadable::validate(nlohmann::json{{"type", "IMU"}}));
    EXPECT_THROW(SensorJsonLoadable::validate(nlohmann::json{{"type", 1}}), std::exception);
    EXPECT_THROW(SensorJsonLoadable::validate(nlohmann::json::object()), std::exception);
}

TEST(json_loadable, schema_loader_throws_for_missing_schema) {
    nlohmann::json schema;
    EXPECT_THROW(sensorbox_schema_loader(nlohmann::json_uri{"file:///DoesNotExist.schema.json"}, schema),
            std::exception);
}

}
