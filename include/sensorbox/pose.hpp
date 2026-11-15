#ifndef SENSORBOX_POSE_HPP
#define SENSORBOX_POSE_HPP

#include <Eigen/Geometry>
#include <mathbox/covariance.hpp>
#include <mathbox/geometry.hpp>
#include <optional>
#include <string>
#include <vector>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/measurement.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

template<int D_>
class DirectPoseSensor : public Sensor, public JsonLoadable<DirectPoseSensorSchemaFilepath, sensorbox_schema_loader> {
public:
    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Degrees of Freedom (DoF)
     *
     */
    static constexpr int DoF = D * (D + 1) / 2;

    /**
     * @brief Covariance matrix
     *
     */
    using Covariance = Eigen::Matrix<double, DoF, DoF>;

    /**
     * @brief Stiffness matrix
     *
     */
    using Stiffness = Eigen::Matrix<double, DoF, DoF>;

    /**
     * @brief Construct an instance of the class from a json config with Sensor structure and optionally one of
     * ["sigma", "sigmas", "variance", "variances", "covariance", "information"]. A sensor whose measurements each
     * carry their own uncertainty has no uncertainty of its own.
     *
     * @param config
     * @param validate
     */
    explicit DirectPoseSensor(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get stiffness matrix for sensor, if the sensor has an uncertainty.
     *
     * @return const std::optional<Stiffness>&
     */
    const std::optional<Stiffness>& stiffness() const;

private:
    std::optional<Stiffness> stiffness_;
};

template<int D_>
class PoseMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;

    explicit PoseMeasurement();

    /**
     * @brief Construct a Pose Measurement, of child_frame_ (C) in frame_ (F), i.e. \f$ \mathbf{T}_{FC} \f$.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     */
    explicit PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_, const std::string& child_frame_,
            const Pose& pose_);

    PoseMeasurement inverse() const;

    void invert();

    const Pose& pose() const;

    Pose& pose();

    /**
     * @brief Create a new pose measurement with a different child frame, as:
     *
     * \f[
     *      T_{F,N} = T_{F,C} * T_{C,N}
     * \f]
     *
     * @param new_child_frame new child frame \f$ N \f$
     * @param T_C_N pose of new child frame in the current child frame \f$ T_{C,N} \f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_child_frame(const std::string& new_child_frame, const Pose& T_C_N) const;

    /**
     * @brief Create a new pose measurement with a different frame, as:
     *
     * \f[
     *      T_{N,C} = T_{N,F} * T_{F,C}
     * \f]
     *
     * @param new_frame new frame \f$ N \f$
     * @param T_N_F pose of current frame in the new frame \f$ T_{N,F} \f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const;

    /**
     * @brief Create a new pose measurement with different frames, as:
     *
     * \f[
     *      T_{NF,NC} = T_{NF,F} * T_{F,C} * T_{C,NC}
     * \f]
     *
     * @param new_frame new frame \f$ NF \f$
     * @param new_child_frame new child frame \f$ NC \f$
     * @param T_NF_F pose of current frame in the new frame \f$ T_{NF,F} \f$
     * @param T_C_NC pose of new child frame in the current child frame \f$ T_{C,NC} \f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_frames(const std::string& new_frame, const std::string& new_child_frame,
            const Pose& T_NF_F, const Pose& T_C_NC) const;

private:
    Pose pose_;
};

template<int D_>
class PoseWithCovarianceMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;
    static constexpr int DoF = D * (D + 1) / 2;
    using Covariance = math::Covariance<double, DoF>;

    explicit PoseWithCovarianceMeasurement();

    /**
     * @brief Construct a Pose Measurement with the covariance of the pose, of child_frame_ (C) in frame_ (F), i.e.
     * \f$ \mathbf{T}_{FC} \f$.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     * @param covariance_ covariance of the pose, of its position and then its orientation, as given by the source of
     * the measurement (e.g. for ROS, of the position x, y, z and the rotations about the x, y and z axes)
     * @param covariance_perturbation_ perturbation of the pose that the covariance is of, if known
     */
    explicit PoseWithCovarianceMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Pose& pose_, const Covariance& covariance_,
            const std::optional<math::PosePerturbation>& covariance_perturbation_ = std::nullopt);

    const Pose& pose() const;

    Pose& pose();

    const Covariance& covariance() const;

    Covariance& covariance();

    /**
     * @brief Get the covariance of another perturbation of the pose, to first order. The perturbation that the
     * covariance of the measurement is of must be known.
     *
     * @param perturbation_
     * @return Covariance
     */
    Covariance covariance(const math::PosePerturbation& perturbation_) const;

    /**
     * @brief Perturbation of the pose that the covariance is of, which is not known unless set, because it is not
     * part of the measurement data.
     *
     * @return const std::optional<math::PosePerturbation>&
     */
    const std::optional<math::PosePerturbation>& covariance_perturbation() const;

    std::optional<math::PosePerturbation>& covariance_perturbation();

private:
    Pose pose_;
    Covariance covariance_;
    std::optional<math::PosePerturbation> covariance_perturbation_;
};

template<int D_>
class PoseMeasurements : public TemporalMeasurement {
public:
    using typename TemporalMeasurement::Clock;
    using typename TemporalMeasurement::Duration;
    using typename TemporalMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;

    explicit PoseMeasurements();

    explicit PoseMeasurements(const Timestamp& timestamp_);

    explicit PoseMeasurements(const Timestamp& timestamp_, const std::vector<PoseMeasurement<D>>& measurements_);

    const std::vector<PoseMeasurement<D>>& measurements() const;

    std::vector<PoseMeasurement<D>>& measurements();

private:
    std::vector<PoseMeasurement<D>> measurements_;
};

}

#include "sensorbox/impl/pose.hpp"

#endif
