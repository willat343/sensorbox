#ifndef SENSORBOX_ACTUATOR_HPP
#define SENSORBOX_ACTUATOR_HPP

#include <cppbox/enum.hpp>
#include <optional>
#include <string>
#include <vector>

#include "sensorbox/current.hpp"
#include "sensorbox/encoder.hpp"
#include "sensorbox/json_loadable.hpp"
#include "sensorbox/measurement.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

CREATE_SMART_ENUM(ActuatorType, DIRECT_DRIVE, GEARED, QUASI_DIRECT_DRIVE, SERIES_ELASTIC, SERVO, STEPPER, UNSPECIFIED)

class Actuator : public Sensor, public JsonLoadable<ActuatorSchemaFilepath, sensorbox_schema_loader> {
public:
    /**
     * @brief Construct an instance of the class from a json config with Actuator structure.
     *
     * @param config
     * @param validate
     */
    explicit Actuator(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get the embedded current sensor, if available.
     *
     * @return const std::optional<CurrentSensor>&
     */
    const std::optional<CurrentSensor>& current_sensor() const;

    /**
     * @brief Get the motor-side encoder, if available.
     *
     * @return const std::optional<Encoder>&
     */
    const std::optional<Encoder>& motor_encoder() const;

    /**
     * @brief Get the joint-side (output-side) encoder, if available.
     *
     * @return const std::optional<Encoder>&
     */
    const std::optional<Encoder>& joint_encoder() const;

    /**
     * @brief Get the actuator type. Note that `Sensor::type()` gives the sensor type.
     *
     * @return ActuatorType
     */
    ActuatorType actuator_type() const;

    /**
     * @brief Get the measurement frequency (Hz) if configured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& frequency() const;

private:
    /**
     * @brief Actuator type.
     *
     */
    ActuatorType actuator_type_;

    /**
     * @brief Measurement frequency (Hz) if configured.
     *
     */
    std::optional<double> frequency_;

    /**
     * @brief Embedded current sensor if available.
     *
     */
    std::optional<CurrentSensor> current_sensor_;

    /**
     * @brief Motor-side encoder if available.
     *
     */
    std::optional<Encoder> motor_encoder_;

    /**
     * @brief Joint-side (output-side) encoder if available.
     *
     */
    std::optional<Encoder> joint_encoder_;

    // TODO: Motor torque constant? (types: all)
    // TODO: Motor gear efficiency [0, 1] (types: GEARED, QUASI_DIRECT_DRIVE, SERVO)
    // TODO: Motor gear ratio (types: GEARED, QUASI_DIRECT_DRIVE, SERVO)
    // TODO: Motor spring stiffness constant (types: SERIES_ELASTIC)
};

class ActuatorMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;

    /**
     * @brief Construct an empty actuator measurement of unspecified type.
     *
     */
    explicit ActuatorMeasurement();

    /**
     * @brief Construct an actuator measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference parent frame of the actuator (parent link)
     * @param child_frame_ reference child frame of the actuator (child link)
     * @param name_ name of the actuator (joint)
     * @param actuator_type_ type of the actuator
     */
    explicit ActuatorMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const std::string& name_, const ActuatorType actuator_type_);

    /**
     * @brief Get the motor current [A], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& current() const;

    /**
     * @brief Get the motor current [A], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& current();

    /**
     * @brief Get the joint-side position [rad], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& joint_position() const;

    /**
     * @brief Get the joint-side position [rad], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& joint_position();

    /**
     * @brief Get the joint-side velocity [rad/s], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& joint_velocity() const;

    /**
     * @brief Get the joint-side velocity [rad/s], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& joint_velocity();

    /**
     * @brief Get the joint-side acceleration [rad/s^2], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& joint_acceleration() const;

    /**
     * @brief Get the joint-side acceleration [rad/s^2], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& joint_acceleration();

    /**
     * @brief Get the joint-side torque [Nm], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& joint_torque() const;

    /**
     * @brief Get the joint-side torque [Nm], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& joint_torque();

    /**
     * @brief Get the motor-side position [rad], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& motor_position() const;

    /**
     * @brief Get the motor-side position [rad], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& motor_position();

    /**
     * @brief Get the motor-side velocity [rad/s], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& motor_velocity() const;

    /**
     * @brief Get the motor-side velocity [rad/s], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& motor_velocity();

    /**
     * @brief Get the motor-side acceleration [rad/s^2], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& motor_acceleration() const;

    /**
     * @brief Get the motor-side acceleration [rad/s^2], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& motor_acceleration();

    /**
     * @brief Get the motor-side torque [Nm], if measured.
     *
     * @return const std::optional<double>&
     */
    const std::optional<double>& motor_torque() const;

    /**
     * @brief Get the motor-side torque [Nm], if measured, to set or reset it.
     *
     * @return std::optional<double>&
     */
    std::optional<double>& motor_torque();

    /**
     * @brief Get the actuator (joint) name.
     *
     * @return const std::string&
     */
    const std::string& name() const;

    /**
     * @brief Get the actuator (joint) name, to set it.
     *
     * @return std::string&
     */
    std::string& name();

    /**
     * @brief Set the actuator type.
     *
     * @param type__ actuator type
     */
    void set_actuator_type(const ActuatorType type__);

    /**
     * @brief Get the actuator type.
     *
     * @return ActuatorType
     */
    ActuatorType actuator_type() const;

private:
    /**
     * @brief Actuator (joint) name
     *
     */
    std::string name_;

    /**
     * @brief Actuator type
     *
     */
    ActuatorType actuator_type_;

    /**
     * @brief Motor current [A]
     *
     * Usually measured by current sensor.
     *
     */
    std::optional<double> current_;

    /**
     * @brief Motor-side position [rad]
     *
     * Usually measured by motor-side encoder.
     *
     */
    std::optional<double> motor_position_;

    /**
     * @brief Motor-side velocity [rad/s]
     *
     * Usually not sensed directly but estimated by numerical differentiation (e.g., finite differencing).
     *
     */
    std::optional<double> motor_velocity_;

    /**
     * @brief Motor-side acceleration [rad/s^2]
     *
     * Usually not sensed directly but estimated by numerical differentiation of the velocity, and often filtered, which
     * attenuates and delays it.
     *
     */
    std::optional<double> motor_acceleration_;

    /**
     * @brief Motor-side torque [Nm]
     *
     * Usually computed as \f$ \tau_m = K \cdot I \f$ where \f$ K \f$ is the motor torque constant [Nm/A] and \f$ I \f$
     * is current. Accuracy depends on quality of current sensing as well as actuator type:
     *  DIRECT_DRIVE: very accurate
     *  GEARED: limited accuracy due to friction and backlash
     *  QUASI_DIRECT_DRIVE: almost as accurate as DIRECT_DRIVE
     *  SERIES_ELASTIC: very accurate
     *  SERVO: very inaccurate and rarely done
     *  STEPPER: very inaccurate and rarely done, torque is non-linear function of phase current
     *
     */
    std::optional<double> motor_torque_;

    /**
     * @brief Joint-side (output-side) position [rad]
     *
     * Usually measured by joint-side encoder.
     *
     */
    std::optional<double> joint_position_;

    /**
     * @brief Joint-side (output-side) velocity [rad/s]
     *
     * Usually not sensed directly but estimated by numerical differentiation (e.g., finite differencing).
     *
     */
    std::optional<double> joint_velocity_;

    /**
     * @brief Joint-side (output-side) acceleration [rad/s^2]
     *
     * Usually not sensed directly but estimated by numerical differentiation of the velocity, and often filtered, which
     * attenuates and delays it.
     *
     */
    std::optional<double> joint_acceleration_;

    /**
     * @brief Joint-side torque [Nm]
     *
     * Usually computed in the following ways for the different actuator types:
     *  DIRECT_DRIVE: \f$ \tau_j = \tau_m \f$, same as motor-side torque
     *  GEARED: \f$ \tau_j \approx \eta \cdot N \cdot \tau_m \f$, where \f$ \eta \f$ is gear efficiency and \f$ N \f$ is
     *      the gear ratio, but additional losses due to friction, backlash and stiction
     *  QUASI_DIRECT_DRIVE: \f$ \tau_j = \eta \cdot N \cdot \tau_m \f$ where \f$ \eta \approx 1 \f$ and usually minimal
     *      losses
     *  SERIES_ELASTIC: \f$ \tau_j = k_s \cdot (\theta_m - \theta_j) \f$ where \f$ k_s \f$ is the spring stiffness
     *      constant and the spring deflection \f$ \theta_s = \theta_m - \theta_j \f$ is the difference between the
     *      motor and joint positions
     *  SERVO: Same as GEARED and rarely done
     *  STEPPER: rarely done, torque is non-linear function of phase current
     *
     */
    std::optional<double> joint_torque_;
};

class ActuatorMeasurements : public TemporalMeasurement {
public:
    using typename TemporalMeasurement::Clock;
    using typename TemporalMeasurement::Duration;
    using typename TemporalMeasurement::Timestamp;

    /**
     * @brief Construct actuator measurements without actuators, at the zero timestamp.
     *
     */
    explicit ActuatorMeasurements();

    /**
     * @brief Construct actuator measurements without actuators.
     *
     * @param timestamp_ timestamp
     */
    explicit ActuatorMeasurements(const Timestamp& timestamp_);

    /**
     * @brief Construct actuator measurements.
     *
     * @param timestamp_ timestamp
     * @param measurements_ measurements of the actuators
     */
    explicit ActuatorMeasurements(const Timestamp& timestamp_, const std::vector<ActuatorMeasurement>& measurements_);

    /**
     * @brief Check if another measurement has as many actuators and, actuator by actuator in order, the same joint
     * position or none.
     *
     * @param other
     * @return true
     * @return false
     */
    bool equal_joint_positions(const ActuatorMeasurements& other) const;

    /**
     * @brief Check if another measurement has as many actuators and, actuator by actuator in order, the same joint
     * velocity or none.
     *
     * @param other
     * @return true
     * @return false
     */
    bool equal_joint_velocities(const ActuatorMeasurements& other) const;

    /**
     * @brief Check if another measurement has as many actuators and, actuator by actuator in order, the same joint
     * acceleration or none.
     *
     * @param other
     * @return true
     * @return false
     */
    bool equal_joint_accelerations(const ActuatorMeasurements& other) const;

    /**
     * @brief Check if another measurement has as many actuators and, actuator by actuator in order, the same joint
     * torque or none.
     *
     * @param other
     * @return true
     * @return false
     */
    bool equal_joint_torques(const ActuatorMeasurements& other) const;

    /**
     * @brief Check if another measurement has as many actuators and, actuator by actuator in order, the same joint
     * positions, velocities, accelerations and torques, each or none (see `equal_joint_positions()`).
     *
     * @param other
     * @return true
     * @return false
     */
    bool equal_joint_measurements(const ActuatorMeasurements& other) const;

    /**
     * @brief Get the joint positions of the actuators, in the order of the measurements.
     *
     * @return std::optional<std::vector<double>> joint positions if every actuator measured one, or `std::nullopt` if
     * none did
     * @throws std::runtime_error if only some actuators measured one
     */
    std::optional<std::vector<double>> joint_positions() const;

    /**
     * @brief Get the joint velocities of the actuators, in the order of the measurements.
     *
     * @return std::optional<std::vector<double>> joint velocities if every actuator measured one, or `std::nullopt` if
     * none did
     * @throws std::runtime_error if only some actuators measured one
     */
    std::optional<std::vector<double>> joint_velocities() const;

    /**
     * @brief Get the joint accelerations of the actuators, in the order of the measurements.
     *
     * @return std::optional<std::vector<double>> joint accelerations if every actuator measured one, or `std::nullopt`
     * if none did
     * @throws std::runtime_error if only some actuators measured one
     */
    std::optional<std::vector<double>> joint_accelerations() const;

    /**
     * @brief Get the joint torques of the actuators, in the order of the measurements.
     *
     * @return std::optional<std::vector<double>> joint torques if every actuator measured one, or `std::nullopt` if
     * none did
     * @throws std::runtime_error if only some actuators measured one
     */
    std::optional<std::vector<double>> joint_torques() const;

    /**
     * @brief Get the measurements of the actuators.
     *
     * @return const std::vector<ActuatorMeasurement>&
     */
    const std::vector<ActuatorMeasurement>& measurements() const;

    /**
     * @brief Get the measurements of the actuators, to modify them.
     *
     * @return std::vector<ActuatorMeasurement>&
     */
    std::vector<ActuatorMeasurement>& measurements();

    /**
     * @brief Get the names of the actuators, in the order of the measurements.
     *
     * @return std::vector<std::string>
     */
    std::vector<std::string> names() const;

    /**
     * @brief Get the measurements of the named actuators, in the order of the names, leaving out the other actuators.
     *
     * @param names_ actuator names
     * @return ActuatorMeasurements at the same timestamp, with the measurement of each named actuator
     * @throws std::runtime_error if a name is not that of exactly one actuator
     */
    ActuatorMeasurements ordered(const std::vector<std::string>& names_) const;

    /**
     * @brief Overwrite the names of the actuators, in the order of the measurements.
     *
     * @param new_names one name for each actuator
     * @throws std::invalid_argument if the number of names is not the number of actuators
     */
    void overwrite_names(const std::vector<std::string>& new_names);

    /**
     * @brief Get the number of actuators.
     *
     * @return std::size_t
     */
    std::size_t size() const;

private:
    /**
     * @brief Measurements of the actuators
     *
     */
    std::vector<ActuatorMeasurement> measurements_;
};

}

#include "sensorbox/impl/actuator.hpp"

#endif
