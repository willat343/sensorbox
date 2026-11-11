#ifndef SENSORBOX_IMPL_POSE_TWIST_HPP
#define SENSORBOX_IMPL_POSE_TWIST_HPP

#include "sensorbox/pose_twist.hpp"

namespace sensorbox {

template<int D_>
inline PoseTwistMeasurement<D_>::PoseTwistMeasurement()
    : PoseTwistMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), Pose::Identity(), Twist::Zero()) {
}

template<int D_>
inline PoseTwistMeasurement<D_>::PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_, const Twist& twist_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_), pose_(pose_), twist_(twist_) {}

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

template<int D_>
inline PoseTwistWithCovarianceMeasurement<D_>::PoseTwistWithCovarianceMeasurement()
    : PoseTwistWithCovarianceMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), Pose::Identity(),
              Twist::Zero(), Covariance(0.0), Covariance(0.0)) {}

template<int D_>
inline PoseTwistWithCovarianceMeasurement<D_>::PoseTwistWithCovarianceMeasurement(const Timestamp& timestamp_,
        const std::string& frame_, const std::string& child_frame_, const Pose& pose_, const Twist& twist_,
        const Covariance& pose_covariance_, const Covariance& twist_covariance_)
    : PoseTwistMeasurement<D_>(timestamp_, frame_, child_frame_, pose_, twist_),
      pose_covariance_(pose_covariance_),
      twist_covariance_(twist_covariance_) {}

template<int D_>
inline auto PoseTwistWithCovarianceMeasurement<D_>::pose_covariance() const -> const Covariance& {
    return pose_covariance_;
}

template<int D_>
inline auto PoseTwistWithCovarianceMeasurement<D_>::pose_covariance() -> Covariance& {
    return const_cast<Covariance&>(std::as_const(*this).pose_covariance());
}

template<int D_>
inline auto PoseTwistWithCovarianceMeasurement<D_>::twist_covariance() const -> const Covariance& {
    return twist_covariance_;
}

template<int D_>
inline auto PoseTwistWithCovarianceMeasurement<D_>::twist_covariance() -> Covariance& {
    return const_cast<Covariance&>(std::as_const(*this).twist_covariance());
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class PoseTwistMeasurement<2>;
extern template class PoseTwistMeasurement<3>;
extern template class PoseTwistWithCovarianceMeasurement<2>;
extern template class PoseTwistWithCovarianceMeasurement<3>;

}
#endif

#endif
