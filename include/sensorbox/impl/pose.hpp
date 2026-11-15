#ifndef SENSORBOX_IMPL_POSE_HPP
#define SENSORBOX_IMPL_POSE_HPP

#include <cassert>
#include <cppbox/exceptions.hpp>

#include "sensorbox/pose.hpp"
#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template<int D_>
inline DirectPoseSensor<D_>::DirectPoseSensor(const nlohmann::json& config, const bool validate)
    : Sensor(config, false),
      JsonLoadable<DirectPoseSensorSchemaFilepath, sensorbox_schema_loader>(config, validate),
      stiffness_(contains_uncertainty(config) ? std::make_optional(stiffness_from_config<DoF>(config)) : std::nullopt) {
    assert(this->type() == SensorType::DIRECT_POSE);
}

template<int D_>
inline auto DirectPoseSensor<D_>::stiffness() const -> const std::optional<Stiffness>& {
    return stiffness_;
}

template<int D_>
inline PoseMeasurement<D_>::PoseMeasurement()
    : PoseMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), Pose::Identity()) {}

template<int D_>
inline PoseMeasurement<D_>::PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const Pose& pose_, const std::optional<Covariance>& covariance_,
        const std::optional<math::PosePerturbation>& covariance_perturbation_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_),
      pose_(pose_),
      covariance_(covariance_),
      covariance_perturbation_(covariance_perturbation_) {}

template<int D_>
inline auto PoseMeasurement<D_>::covariance() const -> const std::optional<Covariance>& {
    return covariance_;
}

template<int D_>
inline auto PoseMeasurement<D_>::covariance() -> std::optional<Covariance>& {
    return const_cast<std::optional<Covariance>&>(std::as_const(*this).covariance());
}

template<int D_>
inline auto PoseMeasurement<D_>::covariance(const math::PosePerturbation& perturbation_) const -> Covariance {
    throw_if(!covariance(), "The pose measurement has no covariance.");
    throw_if(!covariance_perturbation(), "The pose measurement's covariance perturbation is not set.");
    const Eigen::Matrix<double, DoF, DoF> J =
            math::pose_perturbation_jacobian(pose(), perturbation_, *covariance_perturbation());
    return Covariance{J * *covariance() * J.transpose(), true};
}

template<int D_>
inline auto PoseMeasurement<D_>::covariance_perturbation() const -> const std::optional<math::PosePerturbation>& {
    return covariance_perturbation_;
}

template<int D_>
inline auto PoseMeasurement<D_>::covariance_perturbation() -> std::optional<math::PosePerturbation>& {
    return const_cast<std::optional<math::PosePerturbation>&>(std::as_const(*this).covariance_perturbation());
}

template<int D_>
inline auto PoseMeasurement<D_>::inverse() const -> PoseMeasurement<D_> {
    return with_new_pose(child_frame(), frame(), pose().inverse(),
            math::PosePerturbation{math::SEType::JOINT, math::ApplicationSide::RIGHT},
            math::PosePerturbation{math::SEType::JOINT, math::ApplicationSide::LEFT});
}

template<int D_>
inline void PoseMeasurement<D_>::invert() {
    *this = inverse();
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
    const math::PosePerturbation joint_left{math::SEType::JOINT, math::ApplicationSide::LEFT};
    return with_new_pose(frame(), new_child_frame, pose() * T_C_N, joint_left, joint_left);
}

template<int D_>
inline auto PoseMeasurement<D_>::transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const
        -> PoseMeasurement {
    const math::PosePerturbation joint_right{math::SEType::JOINT, math::ApplicationSide::RIGHT};
    return with_new_pose(new_frame, child_frame(), T_N_F * pose(), joint_right, joint_right);
}

template<int D_>
inline auto PoseMeasurement<D_>::transform_to_new_frames(const std::string& new_frame,
        const std::string& new_child_frame, const Pose& T_NF_F, const Pose& T_C_NC) const -> PoseMeasurement {
    return transform_to_new_frame(new_frame, T_NF_F).transform_to_new_child_frame(new_child_frame, T_C_NC);
}

template<int D_>
inline auto PoseMeasurement<D_>::with_new_pose(const std::string& frame_, const std::string& child_frame_,
        const Pose& pose_, const math::PosePerturbation& perturbation,
        const math::PosePerturbation& new_perturbation) const -> PoseMeasurement {
    PoseMeasurement measurement{timestamp(), frame_, child_frame_, pose_};
    if (covariance() && covariance_perturbation()) {
        // Covariance of the new pose's perturbation, converted to the perturbation of this measurement's covariance
        measurement.covariance() = covariance(perturbation);
        measurement.covariance_perturbation() = new_perturbation;
        measurement.covariance() = measurement.covariance(*covariance_perturbation());
        measurement.covariance_perturbation() = covariance_perturbation();
    }
    return measurement;
}

template<int D_>
inline PoseMeasurements<D_>::PoseMeasurements() : PoseMeasurements(Timestamp{Duration::zero()}) {}

template<int D_>
inline PoseMeasurements<D_>::PoseMeasurements(const Timestamp& timestamp_) : TemporalMeasurement(timestamp_) {}

template<int D_>
inline PoseMeasurements<D_>::PoseMeasurements(const Timestamp& timestamp_,
        const std::vector<PoseMeasurement<D>>& measurements_)
    : TemporalMeasurement(timestamp_), measurements_(measurements_) {}

template<int D_>
inline auto PoseMeasurements<D_>::measurements() const -> const std::vector<PoseMeasurement<D>>& {
    return measurements_;
}

template<int D_>
inline auto PoseMeasurements<D_>::measurements() -> std::vector<PoseMeasurement<D>>& {
    return const_cast<std::vector<PoseMeasurement<D>>&>(std::as_const(*this).measurements());
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class DirectPoseSensor<2>;
extern template class DirectPoseSensor<3>;
extern template class JsonLoadable<DirectPoseSensorSchemaFilepath, sensorbox_schema_loader>;

extern template class PoseMeasurement<2>;
extern template class PoseMeasurement<3>;

extern template class PoseMeasurements<2>;
extern template class PoseMeasurements<3>;

}
#endif

#endif
