#ifndef SENSORBOX_POSITION_HPP
#define SENSORBOX_POSITION_HPP

#include <Eigen/Geometry>
#include <mathbox/covariance.hpp>
#include <optional>
#include <string>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/measurement.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

template<int D_>
class PositionSensor : public Sensor, public JsonLoadable<PositionSensorSchemaFilepath, sensorbox_schema_loader> {
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
    static constexpr int DoF = D;

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
     * @brief Construct an instance of the class from a json config with Sensor structure and one of ["sigma", "sigmas",
     * "variance", "variances", "covariance"].
     *
     * @param config
     * @param validate
     */
    explicit PositionSensor(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Get stiffness matrix for sensor.
     *
     * @return const Stiffness&
     */
    const Stiffness& stiffness() const;

private:
    Stiffness stiffness_;
};

template<int D_>
class PositionMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;
    static constexpr int D = D_;
    using Position = Eigen::Vector<double, D>;
    using RotationMatrix = Eigen::Matrix<double, D, D>;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;
    static constexpr int DoF = D;
    using Covariance = math::Covariance<double, DoF>;

    explicit PositionMeasurement();

    /**
     * @brief Construct a Position Measurement, of child_frame_ (C) in frame_ (F) expressed in frame_ (F), i.e.
     * \f$ {}_{F}\mathbf{p}_{FC} \f$.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param position_ position of child_frame_ in frame_ expressed in frame_, \f$ {}_{F}\mathbf{p}_{FC} \f$
     * @param covariance_ covariance of the position, expressed in frame_, as given by the source of the measurement,
     * if any
     */
    explicit PositionMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Position& position_,
            const std::optional<Covariance>& covariance_ = std::nullopt);

    /**
     * @brief Covariance of the position, expressed in the frame of the measurement, if the measurement has one.
     *
     * @return const std::optional<Covariance>&
     */
    const std::optional<Covariance>& covariance() const;

    std::optional<Covariance>& covariance();

    /**
     * @brief Return \f$ {}_{C}\mathbf{p}_{CF} = - \mathbf{R}_{CF} \cdot {}_{F}\mathbf{p}_{FC} \f$, with any
     * covariance rotated to the child frame (the rotation is taken to be exact).
     *
     * @param R_C_F Rotation from child frame C to frame F, \f$ \mathbf{R}_{C,F} \f$.
     * @return PositionMeasurement
     */
    PositionMeasurement inverse(const RotationMatrix& R_C_F) const;

    /**
     * @brief In place `inverse()`.
     *
     * @param R_C_F Rotation from child frame C to frame F, \f$ \mathbf{R}_{C,F} \f$.
     */
    void invert(const RotationMatrix& R_C_F);

    const Position& position() const;

    Position& position();

    /**
     * @brief Create a new pose measurement with a different child frame, as:
     *
     * \f[
     *      {}_{F}\mathbf{p}_{F,N} = {}_{F}\mathbf{p}_{F,C} + {}_{F}\mathbf{p}_{C,N}
     *                             = {}_{F}\mathbf{p}_{F,C} + \mathbf{R}_{F,C} \cdot {}_{C}\mathbf{p}_{C,N}
     * \f]
     *
     * The new measurement has no covariance, because the covariance of the position of the new child frame depends
     * on the uncertainty of the orientation, which the measurement does not have.
     *
     * @param new_child_frame new child frame \f$ N \f$
     * @param R_F_C Rotation from frame F to child frame C, \f$ \mathbf{R}_{C,F} \f$.
     * @param C_p_C_N position of new child frame in the current child frame \f$ {}_{C}\mathbf{p}_{C,N} \f$
     * @return PositionMeasurement
     */
    PositionMeasurement transform_to_new_child_frame(const std::string& new_child_frame, const RotationMatrix& R_F_C,
            const Position& C_p_C_N) const;

    /**
     * @brief Create a new pose measurement with a different frame, as:
     *
     * \f[
     *      {}_{N}\mathbf{p}_{N,C} = \mathbf{T}_{N,F} \cdot {}_{F}\mathbf{p}_{F,C}
     *                             = \mathbf{R}_{N,F} \cdot {}_{F}\mathbf{p}_{F,C} + {}_{N}\mathbf{p}_{N,F}
     * \f]
     *
     * Any covariance is rotated to the new frame.
     *
     * @param new_frame new frame \f$ N \f$
     * @param T_N_F pose of current frame in the new frame \f$ T_{N,F} \f$
     * @return PositionMeasurement
     */
    PositionMeasurement transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const;

    /**
     * @brief Create a new pose measurement with different frames, as:
     *
     * \f[
     *      {}_{NF}\mathbf{p}_{NF,NC} = T_{NF,F} \cdot ({}_{F}\mathbf{p}_{F,C} + {}_{F}\mathbf{p}_{C,NC})
     *          = T_{NF,F} \cdot ({}_{F}\mathbf{p}_{F,C} + \mathbf{R}_{F,C} \cdot {}_{C}\mathbf{p}_{C,NC})
     * \f]
     *
     * The new measurement has no covariance (see `transform_to_new_child_frame`).
     *
     * @param new_frame new frame \f$ NF \f$
     * @param new_child_frame new child frame \f$ NC \f$
     * @param T_NF_F pose of current frame in the new frame \f$ T_{NF,F} \f$
     * @param R_F_C Rotation from frame F to child frame C, \f$ \mathbf{R}_{C,F} \f$.
     * @param C_p_C_N position of new child frame in the current child frame \f$ {}_{C}\mathbf{p}_{C,N} \f$
     * @return PositionMeasurement
     */
    PositionMeasurement transform_to_new_frames(const std::string& new_frame, const std::string& new_child_frame,
            const Pose& T_NF_F, const RotationMatrix& R_F_C, const Position& C_p_C_N) const;

private:
    Position position_;
    std::optional<Covariance> covariance_;
};

}

#include "sensorbox/impl/position.hpp"

#endif
