#ifndef SENSORBOX_IMPL_POSE_TWIST_HPP
#define SENSORBOX_IMPL_POSE_TWIST_HPP

#include "sensorbox/pose_twist.hpp"

namespace sensorbox {

template<int D_>
PoseTwistMeasurement<D_>::PoseTwistMeasurement()
    : PoseTwistMeasurement(Timestamp(Duration::zero()), std::string(), std::string(), Pose::Identity(), Twist::Zero()) {
}

template<int D_>
PoseTwistMeasurement<D_>::PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_, const Twist& twist_)
    : UnaryMeasurement(timestamp_, frame_), child_frame_(child_frame_), pose_(pose_), twist_(twist_) {}

template<int D_>
const std::string& PoseTwistMeasurement<D_>::child_frame() const {
    return child_frame_;
}

template<int D_>
std::string& PoseTwistMeasurement<D_>::child_frame() {
    return const_cast<std::string&>(std::as_const(*this).child_frame());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose() const -> const Pose& {
    return pose_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose() -> Pose& {
    return const_cast<Pose&>(std::as_const(*this).pose());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist() const -> const Twist& {
    return twist_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist() -> Twist& {
    return const_cast<Twist&>(std::as_const(*this).twist());
}

}

#endif
