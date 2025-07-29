#ifndef SENSORBOX_IMPL_POSE_HPP
#define SENSORBOX_IMPL_POSE_HPP

#include <mathbox/stiffness.hpp>

#include "sensorbox/pose.hpp"
#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template<int D_>
DirectPoseSensor<D_>::DirectPoseSensor(const nlohmann::json& config)
    : Sensor(config), stiffness_(stiffness_from_config<DoF>(config)) {
    assert(this->type() == SensorType::DIRECT_POSE);
}

template<int D_>
inline const typename DirectPoseSensor<D_>::Stiffness& DirectPoseSensor<D_>::stiffness() const {
    return stiffness_;
}

template<int D_>
PoseMeasurement<D_>::PoseMeasurement()
    : PoseMeasurement(Timestamp(Duration::zero()), std::string(), std::string(), Pose::Identity()) {}

template<int D_>
PoseMeasurement<D_>::PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_)
    : UnaryMeasurement(timestamp_, frame_), child_frame_(child_frame_), pose_(pose_) {}

template<int D_>
const std::string& PoseMeasurement<D_>::child_frame() const {
    return child_frame_;
}

template<int D_>
std::string& PoseMeasurement<D_>::child_frame() {
    return const_cast<std::string&>(std::as_const(*this).child_frame());
}

template<int D_>
inline auto PoseMeasurement<D_>::pose() const -> const Pose& {
    return pose_;
}

template<int D_>
inline auto PoseMeasurement<D_>::pose() -> Pose& {
    return const_cast<Pose&>(std::as_const(*this).pose());
}

template<int D_>
inline auto PoseMeasurement<D_>::transform_to_new_child_frame(const std::string& new_child_frame,
        const Pose& T_C_N) const -> PoseMeasurement {
    return PoseMeasurement(timestamp(), frame(), new_child_frame, pose() * T_C_N);
}

template<int D_>
inline auto PoseMeasurement<D_>::transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const
        -> PoseMeasurement {
    return PoseMeasurement(timestamp(), new_frame, child_frame(), T_N_F * pose());
}

template<int D_>
inline auto PoseMeasurement<D_>::transform_to_new_frames(const std::string& new_frame,
        const std::string& new_child_frame, const Pose& T_NF_F, const Pose& T_C_NC) const -> PoseMeasurement {
    return PoseMeasurement(timestamp(), new_frame, new_child_frame, T_NF_F * pose() * T_C_NC);
}

}

#endif
