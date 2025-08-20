#ifndef SENSORBOX_IMPL_ENCODER_HPP
#define SENSORBOX_IMPL_ENCODER_HPP

#include "sensorbox/encoder.hpp"
#include "sensorbox/stiffness.hpp"

namespace sensorbox {

inline Encoder::Encoder(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<EncoderSchemaFilepath, sensorbox_schema_loader>(config, validate),
      stiffness_(stiffness_from_config<1>(config)) {}

inline auto Encoder::stiffness() const -> const Stiffness {
    return stiffness_;
}

}

#endif
