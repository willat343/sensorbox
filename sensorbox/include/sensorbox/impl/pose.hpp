#ifndef SENSORBOX_IMPL_POSE_HPP
#define SENSORBOX_IMPL_POSE_HPP

#include "sensorbox/pose.hpp"

namespace sensorbox {

template<int D>
PoseMeasurement<D>::PoseMeasurement()
    : PoseMeasurement(Timestamp(Duration::zero()), std::string(), std::string(), Pose::Identity()) {}

template<int D>
PoseMeasurement<D>::PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_)
    : UnaryMeasurement(timestamp_, frame_, child_frame_), pose_(pose_) {}

template<int D>
inline auto PoseMeasurement<D>::pose() const -> const Pose& {
    return pose_;
}

template<int D>
inline auto PoseMeasurement<D>::pose() -> Pose& {
    return const_cast<Pose&>(std::as_const(*this).pose());
}

template<int D>
inline auto PoseMeasurement<D>::transform_to_new_child_frame(const std::string& new_child_frame,
        const Pose& T_C_N) const -> PoseMeasurement {
    return PoseMeasurement(timestamp(), frame(), new_child_frame, pose() * T_C_N);
}

template<int D>
inline auto PoseMeasurement<D>::transform_to_new_frame(const std::string& new_frame,
        const Pose& T_N_F) const -> PoseMeasurement {
    return PoseMeasurement(timestamp(), new_frame, child_frame(), T_N_F * pose());
}

template<int D>
inline auto PoseMeasurement<D>::transform_to_new_frames(const std::string& new_frame,
        const std::string& new_child_frame, const Pose& T_NF_F, const Pose& T_C_NC) const -> PoseMeasurement {
    return PoseMeasurement(timestamp(), new_frame, new_child_frame, T_NF_F * pose() * T_C_NC);
}

}

#endif
