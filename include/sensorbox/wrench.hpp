#ifndef SENSORBOX_WRENCH_HPP
#define SENSORBOX_WRENCH_HPP

#include <Eigen/Core>
#include <cppbox/enum.hpp>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "sensorbox/measurement.hpp"

namespace sensorbox {

CREATE_SMART_ENUM(WrenchTypeBase, FORCE, TORQUE, WRENCH)

/**
 * @brief Wrench type, i.e. which components of a wrench \f$ \mathbf{w} = [\mathbf{f}^T~\boldsymbol{\tau}^T]^T \f$ are
 * present: the force (FORCE), the torque (TORQUE) or both (WRENCH).
 *
 */
class WrenchType : public WrenchTypeBase {
public:
    /**
     * @brief Construct the default wrench type.
     *
     */
    WrenchType();

    /**
     * @brief Construct a wrench type from its identifier.
     *
     * @param identifier
     */
    WrenchType(const WrenchTypeBase::Identifiers identifier);

    /**
     * @brief Construct a wrench type from its name, e.g. "FORCE".
     *
     * @param string
     */
    WrenchType(const std::string& string);

    /**
     * @brief Get the degrees of freedom of the wrench type in dimension D, i.e. its number of components, D for a
     * force, D(D-1)/2 for a torque and D(D+1)/2 for a wrench (3, 3 and 6 in 3D, and 2, 1 and 3 in 2D).
     *
     * @tparam D dimension
     * @return int
     */
    template<int D>
    int dof() const;

    /**
     * @brief Get the index of the first component of the wrench type in dimension D within the wrench
     * \f$ \mathbf{w} = [\mathbf{f}^T~\boldsymbol{\tau}^T]^T \f$, 0 for a force or a wrench and D for a torque, such
     * that its components are the `dof<D>()` components from this offset.
     *
     * @tparam D dimension
     * @return int
     */
    template<int D>
    int offset() const;
};

/**
 * @brief Measurement of a wrench \f$ \mathbf{w} = [\mathbf{f}^T~\boldsymbol{\tau}^T]^T \f$ acting at the origin of the
 * child frame (C), with the torque about that origin, expressed in the axes of the frame (F). The force and the torque
 * are each optional, and which are present gives the wrench type.
 *
 * For a wrench expressed in the frame at whose origin it acts (e.g. that of a force-torque sensor), the frame and the
 * child frame are the same. A contact force at a foot may instead be expressed in the axes of a world frame.
 *
 * @tparam D_ dimension
 */
template<int D_>
class WrenchMeasurement : public TemporalSpatialRelationalMeasurement {
public:
    using typename TemporalSpatialRelationalMeasurement::Clock;
    using typename TemporalSpatialRelationalMeasurement::Duration;
    using typename TemporalSpatialRelationalMeasurement::Timestamp;

    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Degrees of Freedom (DoF) of the force
     *
     */
    static constexpr int ForceDoF = D;

    /**
     * @brief Degrees of Freedom (DoF) of the torque
     *
     */
    static constexpr int TorqueDoF = D * (D - 1) / 2;

    /**
     * @brief Force vector
     *
     */
    using Force = Eigen::Vector<double, ForceDoF>;

    /**
     * @brief Torque vector
     *
     */
    using Torque = Eigen::Vector<double, TorqueDoF>;

    /**
     * @brief Construct an empty wrench measurement, without a force or a torque. The timestamp is initialised to zero
     * and the frames are left empty.
     *
     */
    explicit WrenchMeasurement();

    /**
     * @brief Construct a wrench measurement, of the wrench acting at the origin of child_frame_ (C) expressed in the
     * axes of frame_ (F).
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in whose axes the wrench is expressed, e.g. world_frame
     * @param child_frame_ reference frame at whose origin the wrench acts, e.g. foot_frame
     * @param force_ force \f$ {}_F\mathbf{f} \f$, if present
     * @param torque_ torque \f$ {}_F\boldsymbol{\tau} \f$ about the origin of child_frame_, if present
     */
    explicit WrenchMeasurement(const Timestamp& timestamp_, const std::string& frame_, const std::string& child_frame_,
            const std::optional<Force>& force_, const std::optional<Torque>& torque_);

    /**
     * @brief Get the force \f$ {}_F\mathbf{f} \f$, if present.
     *
     * @return const std::optional<Force>&
     */
    const std::optional<Force>& force() const;

    /**
     * @brief Get the force \f$ {}_F\mathbf{f} \f$, if present, for modification.
     *
     * @return std::optional<Force>&
     */
    std::optional<Force>& force();

    /**
     * @brief Get the torque \f$ {}_F\boldsymbol{\tau} \f$ about the origin of the child frame, if present.
     *
     * @return const std::optional<Torque>&
     */
    const std::optional<Torque>& torque() const;

    /**
     * @brief Get the torque \f$ {}_F\boldsymbol{\tau} \f$ about the origin of the child frame, if present, for
     * modification.
     *
     * @return std::optional<Torque>&
     */
    std::optional<Torque>& torque();

    /**
     * @brief Get the wrench type, from which of the force and the torque are present. Throws if neither is present.
     *
     * @return WrenchType
     */
    WrenchType type() const;

private:
    /**
     * @brief Force [N], if present.
     *
     */
    std::optional<Force> force_;

    /**
     * @brief Torque [Nm] about the origin of the child frame, if present.
     *
     */
    std::optional<Torque> torque_;
};

/**
 * @brief Wrench measurements at the same time, e.g. of the wrenches acting at the contacts of a robot.
 *
 * @tparam D_ dimension
 */
template<int D_>
class WrenchMeasurements : public TemporalMeasurement {
public:
    using typename TemporalMeasurement::Clock;
    using typename TemporalMeasurement::Duration;
    using typename TemporalMeasurement::Timestamp;

    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Construct empty wrench measurements. The timestamp is initialised to zero.
     *
     */
    explicit WrenchMeasurements();

    /**
     * @brief Construct empty wrench measurements.
     *
     * @param timestamp_ timestamp
     */
    explicit WrenchMeasurements(const Timestamp& timestamp_);

    /**
     * @brief Construct wrench measurements.
     *
     * @param timestamp_ timestamp
     * @param measurements_ wrench measurements
     */
    explicit WrenchMeasurements(const Timestamp& timestamp_, const std::vector<WrenchMeasurement<D>>& measurements_);

    /**
     * @brief Get the wrench measurements.
     *
     * @return const std::vector<WrenchMeasurement<D>>&
     */
    const std::vector<WrenchMeasurement<D>>& measurements() const;

    /**
     * @brief Get the wrench measurements for modification.
     *
     * @return std::vector<WrenchMeasurement<D>>&
     */
    std::vector<WrenchMeasurement<D>>& measurements();

    /**
     * @brief Get the number of wrench measurements.
     *
     * @return std::size_t
     */
    std::size_t size() const;

private:
    /**
     * @brief Wrench measurements.
     *
     */
    std::vector<WrenchMeasurement<D>> measurements_;
};

}

#include "sensorbox/impl/wrench.hpp"

#endif
