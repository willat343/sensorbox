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
    static constexpr int DoF = math::SEDoF<D>;

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
    static constexpr int DoF = math::SEDoF<D>;
    using Covariance = math::Covariance<double, DoF>;

    explicit PoseMeasurement();

    /**
     * @brief Construct a Pose Measurement, of child_frame_ (C) in frame_ (F), i.e. \f$ \mathbf{T}_{FC} \f$.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$ \mathbf{T}_{FC} \f$
     * @param covariance_ covariance of the pose, of its position and then its orientation, as given by the source of
     * the measurement (e.g. for ROS, of the position x, y, z and the rotations about the x, y and z axes), if any
     * @param covariance_perturbation_ perturbation of the pose that the covariance is of, if known
     */
    explicit PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_, const std::string& child_frame_,
            const Pose& pose_, const std::optional<Covariance>& covariance_ = std::nullopt,
            const std::optional<math::PosePerturbation>& covariance_perturbation_ = std::nullopt);

    /**
     * @brief Covariance of the pose, if the measurement has one.
     *
     * @return const std::optional<Covariance>&
     */
    const std::optional<Covariance>& covariance() const;

    std::optional<Covariance>& covariance();

    /**
     * @brief Get the covariance of another perturbation of the pose, to first order. The measurement must have a
     * covariance, and the perturbation that it is of must be known.
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

    /**
     * @brief Create the inverse pose measurement \f$ \mathbf{T}_{CF} = \mathbf{T}_{FC}^{-1} \f$.
     *
     * See `with_new_pose` for its covariance.
     *
     * @return PoseMeasurement
     */
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
     * See `with_new_pose` for its covariance.
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
     * See `with_new_pose` for its covariance.
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
     * See `with_new_pose` for its covariance.
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
    /**
     * @brief Create a measurement of a new pose, obtained from this pose by an operation (an inverse or a composition
     * with another pose) under which a perturbation of this pose is also a perturbation of the new pose, with the same
     * covariance. Only JOINT perturbations have this property: for the tangent vector \f$ \bm{\tau} \f$,
     *  - \f$ (\text{Exp}(\bm{\tau}) \mathbf{T}) \mathbf{C} = \text{Exp}(\bm{\tau}) (\mathbf{T} \mathbf{C}) \f$
     * (LEFT stays LEFT under composition on the right)
     *  - \f$ \mathbf{N} (\mathbf{T} \text{Exp}(\bm{\tau})) = (\mathbf{N} \mathbf{T}) \text{Exp}(\bm{\tau}) \f$
     * (RIGHT stays RIGHT under composition on the left)
     *  - \f$ (\mathbf{T} \text{Exp}(\bm{\tau}))^{-1} = \text{Exp}(-\bm{\tau}) \mathbf{T}^{-1} \f$ (RIGHT becomes LEFT
     * under inversion)
     *
     * The covariance of the new measurement is of the same perturbation as the covariance of this measurement. The new
     * measurement has a covariance only if this measurement has one and the perturbation that it is of is known,
     * because otherwise it is not known how the covariance changes with the pose.
     *
     * @param frame_ frame of the new measurement
     * @param child_frame_ child frame of the new measurement
     * @param pose_ new pose
     * @param perturbation perturbation of this pose
     * @param new_perturbation perturbation of the new pose that the perturbation of this pose is under the operation
     * @return PoseMeasurement
     */
    PoseMeasurement with_new_pose(const std::string& frame_, const std::string& child_frame_, const Pose& pose_,
            const math::PosePerturbation& perturbation, const math::PosePerturbation& new_perturbation) const;

    Pose pose_;
    std::optional<Covariance> covariance_;
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
