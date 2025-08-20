#ifndef SENSORBOX_IMPL_ACTUATOR_HPP
#define SENSORBOX_IMPL_ACTUATOR_HPP

#include "sensorbox/actuator.hpp"

namespace sensorbox {

inline Actuator::Actuator(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<ActuatorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      type_(config["actuator_type"].get<std::string>()),
      current_sensor_(config.contains("current_sensor")
                              ? std::make_optional<CurrentSensor>(config["current_sensor"], false)
                              : std::nullopt),
      motor_encoder_(config.contains("motor_encoder") ? std::make_optional<Encoder>(config["motor_encoder"], false)
                                                      : std::nullopt),
      joint_encoder_(config.contains("joint_encoder") ? std::make_optional<Encoder>(config["joint_encoder"], false)
                                                      : std::nullopt) {}

inline const std::optional<CurrentSensor>& Actuator::current_sensor() const {
    return current_sensor_;
}

inline const std::optional<Encoder>& Actuator::motor_encoder() const {
    return motor_encoder_;
}

inline const std::optional<Encoder>& Actuator::joint_encoder() const {
    return joint_encoder_;
}

inline const ActuatorType Actuator::type() const {
    return type_;
}

inline ActuatorMeasurement::ActuatorMeasurement()
    : ActuatorMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), std::string(),
              ActuatorType::UNSPECIFIED) {}

inline ActuatorMeasurement::ActuatorMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const std::string& name_, const ActuatorType type_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_), name_(name_), type_(type_) {}

inline const std::optional<double>& ActuatorMeasurement::current() const {
    return current_;
}

inline std::optional<double>& ActuatorMeasurement::current() {
    return const_cast<std::optional<double>&>(std::as_const(*this).current());
}

inline const std::optional<double>& ActuatorMeasurement::joint_position() const {
    return joint_position_;
}

inline std::optional<double>& ActuatorMeasurement::joint_position() {
    return const_cast<std::optional<double>&>(std::as_const(*this).joint_position());
}

inline const std::optional<double>& ActuatorMeasurement::joint_velocity() const {
    return joint_velocity_;
}

inline std::optional<double>& ActuatorMeasurement::joint_velocity() {
    return const_cast<std::optional<double>&>(std::as_const(*this).joint_velocity());
}

inline const std::optional<double>& ActuatorMeasurement::joint_torque() const {
    return joint_torque_;
}

inline std::optional<double>& ActuatorMeasurement::joint_torque() {
    return const_cast<std::optional<double>&>(std::as_const(*this).joint_torque());
}

inline const std::optional<double>& ActuatorMeasurement::motor_position() const {
    return motor_position_;
}

inline std::optional<double>& ActuatorMeasurement::motor_position() {
    return const_cast<std::optional<double>&>(std::as_const(*this).motor_position());
}

inline const std::optional<double>& ActuatorMeasurement::motor_velocity() const {
    return motor_velocity_;
}

inline std::optional<double>& ActuatorMeasurement::motor_velocity() {
    return const_cast<std::optional<double>&>(std::as_const(*this).motor_velocity());
}

inline const std::optional<double>& ActuatorMeasurement::motor_torque() const {
    return motor_torque_;
}

inline std::optional<double>& ActuatorMeasurement::motor_torque() {
    return const_cast<std::optional<double>&>(std::as_const(*this).motor_torque());
}

inline const std::string& ActuatorMeasurement::name() const {
    return name_;
}

inline std::string& ActuatorMeasurement::name() {
    return const_cast<std::string&>(std::as_const(*this).name());
}

inline void ActuatorMeasurement::set_type(const ActuatorType type__) {
    type_ = type__;
}

inline const ActuatorType ActuatorMeasurement::type() const {
    return type_;
}

}

#endif
