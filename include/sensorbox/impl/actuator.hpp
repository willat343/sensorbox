#ifndef SENSORBOX_IMPL_ACTUATOR_HPP
#define SENSORBOX_IMPL_ACTUATOR_HPP

#include <algorithm>
#include <cppbox/exceptions.hpp>
#include <ranges>

#include "sensorbox/actuator.hpp"

namespace sensorbox {

inline Actuator::Actuator(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<ActuatorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      actuator_type_(config["actuator_type"].get<std::string>()),
      frequency_(config.contains("frequency") ? std::make_optional(config["frequency"].get<double>()) : std::nullopt),
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

inline ActuatorType Actuator::actuator_type() const {
    return actuator_type_;
}

inline const std::optional<double>& Actuator::frequency() const {
    return frequency_;
}

inline ActuatorMeasurement::ActuatorMeasurement()
    : ActuatorMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), std::string(),
              ActuatorType::UNSPECIFIED) {}

inline ActuatorMeasurement::ActuatorMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const std::string& name_, const ActuatorType actuator_type_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_),
      name_(name_),
      actuator_type_(actuator_type_) {}

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

inline void ActuatorMeasurement::set_actuator_type(const ActuatorType type__) {
    actuator_type_ = type__;
}

inline ActuatorType ActuatorMeasurement::actuator_type() const {
    return actuator_type_;
}

inline ActuatorMeasurements::ActuatorMeasurements() : ActuatorMeasurements(Timestamp{Duration::zero()}) {}

inline ActuatorMeasurements::ActuatorMeasurements(const Timestamp& timestamp_) : TemporalMeasurement(timestamp_) {}

inline ActuatorMeasurements::ActuatorMeasurements(const Timestamp& timestamp_,
        const std::vector<ActuatorMeasurement>& measurements_)
    : TemporalMeasurement(timestamp_), measurements_(measurements_) {}

inline bool ActuatorMeasurements::equal_joint_positions(const ActuatorMeasurements& other) const {
    return size() == other.size() &&
           std::equal(measurements().cbegin(), measurements().cend(), other.measurements().cbegin(),
                   [](const ActuatorMeasurement& lhs, const ActuatorMeasurement& rhs) -> bool {
                       return lhs.joint_position() == rhs.joint_position();
                   });
}

inline bool ActuatorMeasurements::equal_joint_velocities(const ActuatorMeasurements& other) const {
    return size() == other.size() &&
           std::equal(measurements().cbegin(), measurements().cend(), other.measurements().cbegin(),
                   [](const ActuatorMeasurement& lhs, const ActuatorMeasurement& rhs) -> bool {
                       return lhs.joint_velocity() == rhs.joint_velocity();
                   });
}

inline bool ActuatorMeasurements::equal_joint_torques(const ActuatorMeasurements& other) const {
    return size() == other.size() &&
           std::equal(measurements().cbegin(), measurements().cend(), other.measurements().cbegin(),
                   [](const ActuatorMeasurement& lhs, const ActuatorMeasurement& rhs) -> bool {
                       return lhs.joint_torque() == rhs.joint_torque();
                   });
}

inline bool ActuatorMeasurements::equal_joint_measurements(const ActuatorMeasurements& other) const {
    return equal_joint_positions(other) && equal_joint_velocities(other) && equal_joint_torques(other);
}

inline std::optional<std::vector<double>> ActuatorMeasurements::joint_positions() const {
    const bool measured = measurements().empty() || measurements().front().joint_position().has_value();
    throw_if(!std::all_of(measurements().cbegin(), measurements().cend(),
                     [measured](const ActuatorMeasurement& measurement) {
                         return measurement.joint_position().has_value() == measured;
                     }),
            "Only some of the actuators measured a joint position.");
    const auto joint_positions_ = measurements() | std::views::transform([](const ActuatorMeasurement& measurement) {
        return *measurement.joint_position();
    });
    return measured ? std::make_optional(std::vector<double>(joint_positions_.begin(), joint_positions_.end()))
                    : std::nullopt;
}

inline std::optional<std::vector<double>> ActuatorMeasurements::joint_velocities() const {
    const bool measured = measurements().empty() || measurements().front().joint_velocity().has_value();
    throw_if(!std::all_of(measurements().cbegin(), measurements().cend(),
                     [measured](const ActuatorMeasurement& measurement) {
                         return measurement.joint_velocity().has_value() == measured;
                     }),
            "Only some of the actuators measured a joint velocity.");
    const auto joint_velocities_ = measurements() | std::views::transform([](const ActuatorMeasurement& measurement) {
        return *measurement.joint_velocity();
    });
    return measured ? std::make_optional(std::vector<double>(joint_velocities_.begin(), joint_velocities_.end()))
                    : std::nullopt;
}

inline std::optional<std::vector<double>> ActuatorMeasurements::joint_torques() const {
    const bool measured = measurements().empty() || measurements().front().joint_torque().has_value();
    throw_if(!std::all_of(measurements().cbegin(), measurements().cend(),
                     [measured](const ActuatorMeasurement& measurement) {
                         return measurement.joint_torque().has_value() == measured;
                     }),
            "Only some of the actuators measured a joint torque.");
    const auto joint_torques_ = measurements() | std::views::transform([](const ActuatorMeasurement& measurement) {
        return *measurement.joint_torque();
    });
    return measured ? std::make_optional(std::vector<double>(joint_torques_.begin(), joint_torques_.end()))
                    : std::nullopt;
}

inline const std::vector<ActuatorMeasurement>& ActuatorMeasurements::measurements() const {
    return measurements_;
}

inline std::vector<ActuatorMeasurement>& ActuatorMeasurements::measurements() {
    return const_cast<std::vector<ActuatorMeasurement>&>(std::as_const(*this).measurements());
}

inline void ActuatorMeasurements::overwrite_names(const std::vector<std::string>& new_names) {
    throw_if(measurements().size() != new_names.size(), "Size mismatch when overwriting names.", std::invalid_argument);
    for (std::size_t i = 0; i < measurements().size(); ++i) {
        measurements()[i].name() = new_names[i];
    }
}

inline std::vector<std::string> ActuatorMeasurements::names() const {
    std::vector<std::string> names_(size());
    std::transform(measurements().cbegin(), measurements().cend(), names_.begin(),
            [](const ActuatorMeasurement& measurement) { return measurement.name(); });
    return names_;
}

inline std::size_t ActuatorMeasurements::size() const {
    return measurements().size();
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class JsonLoadable<ActuatorSchemaFilepath, sensorbox_schema_loader>;

}
#endif

#endif
