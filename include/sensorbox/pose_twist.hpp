#ifndef SENSORBOX_POSE_TWIST_HPP
#define SENSORBOX_POSE_TWIST_HPP

#include <Eigen/Geometry>
#include <mathbox/covariance.hpp>
#include <mathbox/geometry.hpp>
#include <optional>
#include <string>

#include "sensorbox/measurement.hpp"

namespace sensorbox {

template<int D_>
class PoseTwistMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;
    static constexpr int PoseDoF = math::SEDoF<D>;
    using Twist = Eigen::Vector<double, PoseDoF>;
    using Covariance = math::Covariance<double, PoseDoF>;

    explicit PoseTwistMeasurement();

    /**
     * @brief Construct a Pose-Twist Measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     * @param twist_ twist of child_frame_ in child_frame_
     * @param pose_covariance_ covariance of the pose, of its position and then its orientation, as given by the source
     * of the measurement (e.g. for ROS, of the position x, y, z and the rotations about the x, y and z axes), if any
     * @param twist_covariance_ covariance of the twist, of its linear and then its angular velocity, as given by the
     * source of the measurement, if any
     * @param pose_covariance_perturbation_ perturbation of the pose that the pose covariance is of, if known
     */
    explicit PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Pose& pose_, const Twist& twist_,
            const std::optional<Covariance>& pose_covariance_ = std::nullopt,
            const std::optional<Covariance>& twist_covariance_ = std::nullopt,
            const std::optional<math::PosePerturbation>& pose_covariance_perturbation_ = std::nullopt);

    const Pose& pose() const;

    Pose& pose();

    /**
     * @brief Covariance of the pose, if the measurement has one.
     *
     * @return const std::optional<Covariance>&
     */
    const std::optional<Covariance>& pose_covariance() const;

    std::optional<Covariance>& pose_covariance();

    /**
     * @brief Get the covariance of another perturbation of the pose, to first order. The measurement must have a pose
     * covariance, and the perturbation that it is of must be known.
     *
     * @param perturbation_
     * @return Covariance
     */
    Covariance pose_covariance(const math::PosePerturbation& perturbation_) const;

    /**
     * @brief Perturbation of the pose that the pose covariance is of, which is not known unless set, because it is not
     * part of the measurement data.
     *
     * @return const std::optional<math::PosePerturbation>&
     */
    const std::optional<math::PosePerturbation>& pose_covariance_perturbation() const;

    std::optional<math::PosePerturbation>& pose_covariance_perturbation();

    const Twist& twist() const;

    Twist& twist();

    /**
     * @brief Covariance of the twist, if the measurement has one.
     *
     * @return const std::optional<Covariance>&
     */
    const std::optional<Covariance>& twist_covariance() const;

    std::optional<Covariance>& twist_covariance();

private:
    Pose pose_;
    Twist twist_;
    std::optional<Covariance> pose_covariance_;
    std::optional<Covariance> twist_covariance_;
    std::optional<math::PosePerturbation> pose_covariance_perturbation_;
};

}

#include "sensorbox/impl/pose_twist.hpp"

#endif
