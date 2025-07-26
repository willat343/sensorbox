#ifndef SENSORBOX_IMU_HPP
#define SENSORBOX_IMU_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <string>

#include "sensorbox/accelerometer.hpp"
#include "sensorbox/gyroscope.hpp"
#include "sensorbox/sensor.hpp"
#include "sensorbox/unary.hpp"

namespace sensorbox {

template<int D_>
class Imu : public Sensor {
public:
    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Construct an instance of the class from a json config with Sensor structure plus:
     * ```json
     * "accelerometer": <Accelerometer>,
     * "gyroscope": <Gyroscope>
     * ```
     *
     * @param config
     */
    explicit Imu(const nlohmann::json& config);

    const Accelerometer<D>& accelerometer() const;

    const Gyroscope<D>& gyroscope() const;

private:
    Accelerometer<D> accelerometer_;
    Gyroscope<D> gyroscope_;
};

template<int D_>
class ImuMeasurement : public UnaryMeasurement {
public:
    using Clock = UnaryMeasurement::Clock;
    using Duration = UnaryMeasurement::Duration;
    using Timestamp = UnaryMeasurement::Timestamp;
    static constexpr int D = D_;
    static constexpr int AccelDoF = D;
    static constexpr int GyroDoF = D * (D - 1) / 2;
    using AngularVelocity = Eigen::Matrix<double, GyroDoF, 1>;
    using LinearAcceleration = Eigen::Matrix<double, AccelDoF, 1>;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;

    explicit ImuMeasurement();

    /**
     * @brief Construct a Imu Measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. imu_frame
     * @param angular_velocity_ angular velocity (rad/s)
     * @param linear_acceleration_ linear acceleration (m/s^2)
     */
    explicit ImuMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const AngularVelocity& angular_velocity_, const LinearAcceleration& linear_acceleration_);

    const AngularVelocity& angular_velocity() const;

    AngularVelocity& angular_velocity();

    const LinearAcceleration& linear_acceleration() const;

    LinearAcceleration& linear_acceleration();

    ImuMeasurement transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const;

private:
    /**
     * @brief The angular velocity of an IMU is \f$\boldsymbol{\omega}^{F}_{IF}\f$ the angular velocity of (body)
     * `frame()` \f$F\f$ with respect to an inertial frame \f$I\f$ expressed in `frame()` \f$F\f$.
     *
     * This can be split into components where \f$E\f$ is the Earth-Centered Earth-Fixed (ECEF) frame and \f$N\f$ is a
     * chosen navigation frame (e.g. ENU or NED at the Earth's surface):
     * \f[
     *      \boldsymbol{\omega}^{F}_{IF} = \boldsymbol{\omega}^{F}_{IE} + \boldsymbol{\omega}^{F}_{EN} +
     * \boldsymbol{\omega}^{F}_{NF} = \mathbf{R}^{FN} (\boldsymbol{\omega}^{N}_{IE} + \boldsymbol{\omega}^{N}_{EN}) +
     * \boldsymbol{\omega}^{F}_{NF}
     * \f]
     *
     * This lets us rewrite it in terms of the angular velocity with respect to the navigation frame
     * \f$\boldsymbol{\omega}^{F}_{NF}\f$:
     * \f[
     *      \boldsymbol{\omega}^{F}_{NF} = \boldsymbol{\omega}^{F}_{IF} - \mathbf{R}^{FN} (\boldsymbol{\omega}^{N}_{IE}
     * + \boldsymbol{\omega}^{N}_{EN})
     * \f]
     *
     */
    AngularVelocity angular_velocity_;

    /**
     * @brief The linear acceleration of an IMU is the "specific force" \f$\mathbf{f}^{F}\f$ expressed in (body)
     * `frame()` \f$F\f$ of the IMU, equal to \f$\mathbf{a}^{F}_{IF} - \mathbf{g}^{F}\f$ where \f$\mathbf{a}^{F}_{IF}\f$
     * is the acceleration of the `frame()` with respect to an inertial reference frame \f$I\f$, expressed in `frame()`,
     * and \f$\mathbf{g}^{F}\f$ is the gravity expressed in `frame()`.
     *
     * One typically estimates for or knows \f$\mathbf{g}^{N}\f$ where \f$N\f$ is a navigation frame and we assume small
     * enough movement along this frame that \f$\mathbf{g}^{N}\f$ does not change. Then:
     * \f[
     *      \mathbf{f}^{F} = \mathbf{R}^{FN} (\mathbf{a}^{N}_{IF} - \mathbf{g}^{N})
     * \f]
     * or rearranged:
     * \f[
     *      \mathbf{a}^{N}_{IF} = \mathbf{R}^{NF} \mathbf{f}^{F} + \mathbf{g}^{N}
     * \f]
     *
     * If we consider an ENU navigation frame (Z-up) then \f$\mathbf{g}^{N} := \begin{bmatrix}0 \\ 0
     * \\ -g\end{bmatrix}\f$ (at the origin). For a NED navigation frame (Z-down) we have \f$\mathbf{g}^{N} :=
     * \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix}\f$ (at the origin). When the IMU frame is oriented with the navigation
     * frame (\f$\mathbf{R}^{FN} = \mathbf{I}\f$) and at rest (\f$\mathbf{a}^{N}_{IF}\f$), then the device will measure
     * \f$\mathbf{f}^F = -\mathbf{g}^{N}\f$. This yields an output \f$[0, 0, +g]\f$ in ENU and \f$[0, 0, -g]\f$ in NED.
     *
     */
    LinearAcceleration linear_acceleration_;
};

}

#include "sensorbox/impl/imu.hpp"

#endif
