#ifndef SENSORBOX_IMPL_POSITION_HPP
#define SENSORBOX_IMPL_POSITION_HPP

#include <mathbox/stiffness.hpp>

#include "sensorbox/position.hpp"
#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template<int D_>
inline DirectPositionSensor<D_>::DirectPositionSensor(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<DirectPositionSensorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      stiffness_(stiffness_from_config<DoF>(config)) {
    assert(this->type() == SensorType::DIRECT_POSITION);
}

template<int D_>
inline const typename DirectPositionSensor<D_>::Stiffness& DirectPositionSensor<D_>::stiffness() const {
    return stiffness_;
}

template<int D_>
inline PositionMeasurement<D_>::PositionMeasurement()
    : PositionMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), Position::Zero()) {}

template<int D_>
inline PositionMeasurement<D_>::PositionMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Position& position_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_), position_(position_) {}

template<int D_>
inline auto PositionMeasurement<D_>::inverse(const RotationMatrix& R_C_F) const -> PositionMeasurement<D_> {
    return PositionMeasurement<D_>{timestamp(), child_frame(), frame(), -R_C_F * position()};
}

template<int D_>
inline void PositionMeasurement<D_>::invert(const RotationMatrix& R_C_F) {
    *this = inverse(R_C_F);
}

template<int D_>
inline auto PositionMeasurement<D_>::position() const -> const Position& {
    return position_;
}

template<int D_>
inline auto PositionMeasurement<D_>::position() -> Position& {
    return const_cast<Position&>(std::as_const(*this).position());
}

template<int D_>
inline auto PositionMeasurement<D_>::transform_to_new_child_frame(const std::string& new_child_frame,
        const RotationMatrix& R_F_C, const Position& C_p_C_N) const -> PositionMeasurement {
    return PositionMeasurement(timestamp(), frame(), new_child_frame, position() + R_F_C * C_p_C_N);
}

template<int D_>
inline auto PositionMeasurement<D_>::transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const
        -> PositionMeasurement {
    return PositionMeasurement(timestamp(), new_frame, child_frame(), T_N_F * position());
}

template<int D_>
inline auto PositionMeasurement<D_>::transform_to_new_frames(const std::string& new_frame,
        const std::string& new_child_frame, const Pose& T_NF_F, const RotationMatrix& R_F_C,
        const Position& C_p_C_N) const -> PositionMeasurement {
    return PositionMeasurement(timestamp(), new_frame, new_child_frame, T_NF_F * (position() + R_F_C * C_p_C_N));
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class DirectPositionSensor<2>;
extern template class DirectPositionSensor<3>;
extern template class JsonLoadable<DirectPositionSensorSchemaFilepath, sensorbox_schema_loader>;

extern template class PositionMeasurement<2>;
extern template class PositionMeasurement<3>;

}
#endif

#endif
