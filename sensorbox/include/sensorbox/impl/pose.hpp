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

}

#endif
