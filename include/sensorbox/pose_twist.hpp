#ifndef SENSORBOX_POSE_TWIST_HPP
#define SENSORBOX_POSE_TWIST_HPP

#include <Eigen/Geometry>
#include <mathbox/covariance.hpp>
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
    static constexpr int PoseDoF = D * (D + 1) / 2;
    using Twist = Eigen::Vector<double, PoseDoF>;

    explicit PoseTwistMeasurement();

    /**
     * @brief Construct a Pose-Twist Measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     * @param twist_ twist of child_frame_ in child_frame_
     */
    explicit PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Pose& pose_, const Twist& twist_);

    const Pose& pose() const;

    Pose& pose();

    const Twist& twist() const;

    Twist& twist();

private:
    Pose pose_;
    Twist twist_;
};

template<int D_>
class PoseTwistWithCovarianceMeasurement : public PoseTwistMeasurement<D_> {
public:
    using typename PoseTwistMeasurement<D_>::Clock;
    using typename PoseTwistMeasurement<D_>::Duration;
    using typename PoseTwistMeasurement<D_>::Timestamp;
    using typename PoseTwistMeasurement<D_>::Pose;
    using typename PoseTwistMeasurement<D_>::Twist;
    static constexpr int D = D_;
    static constexpr int PoseDoF = PoseTwistMeasurement<D_>::PoseDoF;
    using Covariance = math::Covariance<double, PoseDoF>;

    explicit PoseTwistWithCovarianceMeasurement();

    /**
     * @brief Construct a Pose-Twist Measurement with the covariances of the pose and of the twist.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     * @param twist_ twist of child_frame_ in child_frame_
     * @param pose_covariance_ covariance of the pose, of its position and then its orientation, as given by the source
     * of the measurement (e.g. for ROS, of the position x, y, z and the rotations about the x, y and z axes)
     * @param twist_covariance_ covariance of the twist, of its linear and then its angular velocity, as given by the
     * source of the measurement
     */
    explicit PoseTwistWithCovarianceMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Pose& pose_, const Twist& twist_, const Covariance& pose_covariance_,
            const Covariance& twist_covariance_);

    const Covariance& pose_covariance() const;

    Covariance& pose_covariance();

    const Covariance& twist_covariance() const;

    Covariance& twist_covariance();

private:
    Covariance pose_covariance_;
    Covariance twist_covariance_;
};

}

#include "sensorbox/impl/pose_twist.hpp"

#endif
