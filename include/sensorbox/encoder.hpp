#ifndef SENSORBOX_ENCODER_HPP
#define SENSORBOX_ENCODER_HPP

#include <Eigen/Core>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

class Encoder : public Sensor, public JsonLoadable<EncoderSchemaFilepath, sensorbox_schema_loader> {
public:
    using Stiffness = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Construct an instance of the class from a json config with Encoder structure.
     *
     * @param config
     * @param validate
     */
    explicit Encoder(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get stiffness for sensor.
     *
     * @return const Stiffness&
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
