#ifndef SENSORBOX_IMPL_POSE_TWIST_HPP
#define SENSORBOX_IMPL_POSE_TWIST_HPP

#include <cppbox/exceptions.hpp>

#include "sensorbox/pose_twist.hpp"

namespace sensorbox {

template<int D_>
inline PoseTwistMeasurement<D_>::PoseTwistMeasurement()
    : PoseTwistMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), Pose::Identity(), Twist::Zero()) {
}

template<int D_>
inline PoseTwistMeasurement<D_>::PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_, const Twist& twist_,
        const std::optional<Covariance>& pose_covariance_, const std::optional<Covariance>& twist_covariance_,
        const std::optional<math::PosePerturbation>& pose_covariance_perturbation_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_),
      pose_(pose_),
      twist_(twist_),
      pose_covariance_(pose_covariance_),
      twist_covariance_(twist_covariance_),
      pose_covariance_perturbation_(pose_covariance_perturbation_) {}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose() const -> const Pose& {
    return pose_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose() -> Pose& {
    return const_cast<Pose&>(std::as_const(*this).pose());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose_covariance() const -> const std::optional<Covariance>& {
    return pose_covariance_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose_covariance() -> std::optional<Covariance>& {
    return const_cast<std::optional<Covariance>&>(std::as_const(*this).pose_covariance());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose_covariance(const math::PosePerturbation& perturbation_) const -> Covariance {
    throw_if(!pose_covariance(), "The pose twist measurement has no pose covariance.");
    throw_if(!pose_covariance_perturbation(), "The pose twist measurement's pose covariance perturbation is not set.");
    const Eigen::Matrix<double, PoseDoF, PoseDoF> J =
            math::pose_perturbation_jacobian(pose(), perturbation_, *pose_covariance_perturbation());
    return Covariance{J * *pose_covariance() * J.transpose(), true};
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose_covariance_perturbation() const
        -> const std::optional<math::PosePerturbation>& {
    return pose_covariance_perturbation_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::pose_covariance_perturbation() -> std::optional<math::PosePerturbation>& {
    return const_cast<std::optional<math::PosePerturbation>&>(std::as_const(*this).pose_covariance_perturbation());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist() const -> const Twist& {
    return twist_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist() -> Twist& {
    return const_cast<Twist&>(std::as_const(*this).twist());
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist_covariance() const -> const std::optional<Covariance>& {
    return twist_covariance_;
}

template<int D_>
inline auto PoseTwistMeasurement<D_>::twist_covariance() -> std::optional<Covariance>& {
    return const_cast<std::optional<Covariance>&>(std::as_const(*this).twist_covariance());
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class PoseTwistMeasurement<2>;
extern template class PoseTwistMeasurement<3>;

}
#endif

#endif
