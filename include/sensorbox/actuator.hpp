#ifndef SENSORBOX_ACTUATOR_HPP
#define SENSORBOX_ACTUATOR_HPP

#include <cppbox/enum.hpp>
#include <optional>

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

    const std::optional<CurrentSensor>& current_sensor() const;

    const std::optional<Encoder>& motor_encoder() const;

    const std::optional<Encoder>& joint_encoder() const;

    const ActuatorType type() const;

private:
    ActuatorType type_;

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
     * @param type_ type of the actuator
     */
    explicit ActuatorMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const std::string& name_, const ActuatorType type_);

    const std::optional<double>& current() const;

    std::optional<double>& current();

    const std::optional<double>& joint_position() const;

    std::optional<double>& joint_position();

    const std::optional<double>& joint_velocity() const;

    std::optional<double>& joint_velocity();

    const std::optional<double>& joint_torque() const;

    std::optional<double>& joint_torque();

    const std::optional<double>& motor_position() const;

    std::optional<double>& motor_position();

    const std::optional<double>& motor_velocity() const;

    std::optional<double>& motor_velocity();

    const std::optional<double>& motor_torque() const;

    std::optional<double>& motor_torque();

    const std::string& name() const;

    std::string& name();

    void set_type(const ActuatorType type__);

    const ActuatorType type() const;

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
    ActuatorType type_;

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
     * Usually not sensed directly but estimated by numerical differentation (e.g., finite differencing).
     *
     */
    std::optional<double> motor_velocity_;

    /**
     * @brief Motor-side torque [Nm]
     *
     * Usually computed as \f$\tau_m = K \cdot I\f$ where \f$K\f$ is the motor torque constant [Nm/A] and \f$I\f$ is
     * current. Accuracy depends on quality of current sensing as well as actuator type:
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
     * Usually not sensed directly but estimated by numerical differentation (e.g., finite differencing).
     *
     */
    std::optional<double> joint_velocity_;

    /**
     * @brief Joint-side torque [Nm]
     *
     * Usually computed in the following ways for the different actuator types:
     *  DIRECT_DRIVE: \f$\tau_j = \tau_m\f$, same as motor-side torque
     *  GEARED: \f$\tau_j \approx \eta \cdot N \cdot \tau_m\f$, where \f$\eta\f$ is gear efficiency and \f$N\f$ is the
     *      gear ratio, but additional losses due to friction, backlash and stiction
     *  QUASI_DIRECT_DRIVE: f$\tau_j = \eta \cdot N \cdot \tau_m\f$ wutg \f$\eta \approx 1\f$ and usually minimal losses
     *  SERIES_ELASTIC: \f$\tau_j = k_s \cdot (\theta_m - \theta_j)\f$ where \f$k_s\f$ is the spring stiffness constant
     *      and the spring deflection \f$\theta_s = \theta_m - \theta_j\f$ is the difference between the motor and joint
     *      positions
     *  SERVO: Same as GEARED and rarely done
     *  STEPPER: rarely done, torque is non-linear function of phase current
     *
     */
    std::optional<double> joint_torque_;
};

}

#include "sensorbox/impl/actuator.hpp"

#endif
