#ifndef SENSORBOX_ENCODER_HPP
#define SENSORBOX_ENCODER_HPP

#include <Eigen/Core>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

class Encoder : public Sensor, public JsonLoadable<EncoderSchemaFilepath, sensorbox_schema_loader> {
public:
    using Covariance = Eigen::Matrix<double, 1, 1>;
    using Stiffness = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Construct an instance of the class from a json config with Encoder structure.
     *
     * @param config
     * @param validate
     */
    explicit Encoder(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get covariance for sensor.
     *
     * TODO: consider replacing covariance/stiffness functions in sensors with uncertainty (capturing all these, to
     * exist in mathbox)
     *
     * @return const Covariance
     */
    const Covariance covariance() const;

    /**
     * @brief Get stiffness for sensor.
     *
     * @return const Stiffness
     */
    const Stiffness stiffness() const;

private:
    /**
     * @brief Stiffness in rad^{-1} of the encoder position readings
     *
     */
    Stiffness stiffness_;
};

}

#include "sensorbox/impl/encoder.hpp"

#endif
