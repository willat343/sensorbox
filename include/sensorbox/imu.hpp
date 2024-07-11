#ifndef EIGEN_ROS_IMU_HPP
#define EIGEN_ROS_IMU_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <string>

namespace sensorbox {

class Imu {
public:
    using TimeStamp = std::chrono::time_point<std::chrono::steady_clock>;

    explicit Imu(const Eigen::Quaterniond& orientation = Eigen::Quaterniond(0, 0, 0, 0),
            const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& linear_acceleration = Eigen::Vector3d::Zero(),
            const Eigen::Matrix3d& orientation_covariance = Eigen::Matrix3d::Zero(),
            const Eigen::Matrix3d& angular_velocity_covariance = Eigen::Matrix3d::Zero(),
            const Eigen::Matrix3d& linear_acceleration_covariance = Eigen::Matrix3d::Zero(),
            const TimeStamp& timestamp = TimeStamp(), const std::string& frame = std::string());

    /**
     * @brief Transform IMU to a new reference frame using a rotation extrinsic. The inverse transform must also be
     * passed to this function, which allows for greater efficiency when this transform can be calculated only once.
     *
     * TODO: The math on the orientation covariance needs to be checked.
     *
     * @param to_frame_ext rotation extrinsic from current (IMU) to new frame
     * @param from_frame_ext rotation extrinsic from new to current (IMU) frame (should equal inverse of to_frame_ext)
     */
    void change_frame(const Eigen::Quaterniond& to_frame_ext, const Eigen::Quaterniond& from_frame_ext);

    std::string to_string(const unsigned int precision = 3) const;

    Eigen::Vector3d rpy() const;

    // Orientation measurement. (0,0,0,0) if not known.
    Eigen::Quaterniond orientation;
    // Angular velocity measurement.
    Eigen::Vector3d angular_velocity;
    // Linear acceleration measurement.
    Eigen::Vector3d linear_acceleration;
    // Orientation covariance.
    Eigen::Matrix3d orientation_covariance;
    // Angular velocity covariance.
    Eigen::Matrix3d angular_velocity_covariance;
    // Linear acceleration covariance.
    Eigen::Matrix3d linear_acceleration_covariance;
    // Measurement timestamp.
    TimeStamp timestamp;
    // Reference frame for measurement.
    std::string frame;
};

bool operator==(const Imu& lhs, const Imu& rhs);

Imu lerp(const Imu& imu1, const Imu& imu2, const typename Imu::TimeStamp& interp_timestamp);

}

#include "sensorbox/impl/imu.hpp"

#endif
