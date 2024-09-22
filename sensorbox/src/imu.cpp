#include "sensorbox/imu.hpp"

#include <iomanip>
#include <iostream>
#include <mathbox/lerp.hpp>

namespace sensorbox {

Imu::Imu(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity,
        const Eigen::Vector3d& linear_acceleration, const Eigen::Matrix3d& orientation_covariance,
        const Eigen::Matrix3d& angular_velocity_covariance, const Eigen::Matrix3d& linear_acceleration_covariance,
        const TimeStamp& timestamp, const std::string& frame)
    : orientation(orientation),
      angular_velocity(angular_velocity),
      linear_acceleration(linear_acceleration),
      orientation_covariance(orientation_covariance),
      angular_velocity_covariance(angular_velocity_covariance),
      linear_acceleration_covariance(linear_acceleration_covariance),
      timestamp(timestamp),
      frame(frame) {}

void Imu::change_frame(const Eigen::Quaterniond& to_frame_ext, const Eigen::Quaterniond& from_frame_ext) {
    const Eigen::Matrix3d to_frame_ext_m = to_frame_ext.matrix();
    const Eigen::Matrix3d to_frame_ext_m_transpose = to_frame_ext_m.transpose();
    const Eigen::Matrix3d from_frame_ext_m = from_frame_ext.matrix();
    const Eigen::Matrix3d from_frame_ext_m_transpose = from_frame_ext_m.transpose();

    // Orientation
    orientation = orientation * to_frame_ext;
    orientation_covariance = to_frame_ext_m * orientation_covariance * to_frame_ext_m_transpose;

    // Angular velocity
    angular_velocity = from_frame_ext * angular_velocity;
    angular_velocity_covariance = from_frame_ext_m * angular_velocity_covariance * from_frame_ext_m_transpose;

    // Linear acceleration
    linear_acceleration = from_frame_ext * linear_acceleration;
    linear_acceleration_covariance = from_frame_ext_m * linear_acceleration_covariance * from_frame_ext_m_transpose;
}

std::string Imu::to_string(const unsigned int precision) const {
    Eigen::Vector3d rpy_ = rpy();
    std::stringstream ss;
    ss << std::setprecision(precision);
    ss << "IMU (" << std::chrono::duration<double>(timestamp.time_since_epoch()).count() << ", " << frame << ")\n"
       << "\torientation:\n"
       << "\t\tquaternion (wxyz): [" << orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", "
       << orientation.z() << "]\n"
       << "\t\tRPY (radians):     [" << rpy_[0] << ", " << rpy_[1] << ", " << rpy_[2] << "]\n"
       << "\t\tRPY (degrees):     [" << 180.0 * rpy_[0] / M_PI << ", " << 180.0 * rpy_[1] / M_PI << ", "
       << 180.0 * rpy_[2] / M_PI << "]\n"
       << "\tangular velocity:    [" << angular_velocity[0] << ", " << angular_velocity[1] << ", "
       << angular_velocity[2] << "] (" << angular_velocity.norm() << ")\n"
       << "\tlinear acceleration: [" << linear_acceleration[0] << ", " << linear_acceleration[1] << ", "
       << linear_acceleration[2] << "] (" << linear_acceleration.norm() << ")\n";
    return ss.str();
}

bool operator==(const Imu& lhs, const Imu& rhs) {
    return lhs.orientation.toRotationMatrix().isApprox(rhs.orientation.toRotationMatrix()) &&
           lhs.angular_velocity.isApprox(rhs.angular_velocity) &&
           lhs.linear_acceleration.isApprox(rhs.linear_acceleration) &&
           lhs.orientation_covariance.isApprox(rhs.orientation_covariance) &&
           lhs.angular_velocity_covariance.isApprox(rhs.angular_velocity_covariance) &&
           lhs.linear_acceleration_covariance.isApprox(rhs.linear_acceleration_covariance) &&
           lhs.timestamp == rhs.timestamp && lhs.frame == rhs.frame;
}

Imu lerp(const Imu& imu1, const Imu& imu2, const Imu::TimeStamp& interp_timestamp) {
    // Check interpolation is possible
    if (interp_timestamp < imu1.timestamp || interp_timestamp > imu2.timestamp) {
        throw std::runtime_error(
                "Imu interpolation error: new_timestamp " +
                std::to_string(std::chrono::duration<double>(interp_timestamp.time_since_epoch()).count()) +
                " was not between imu timestamps " +
                std::to_string(std::chrono::duration<double>(imu1.timestamp.time_since_epoch()).count()) + " and " +
                std::to_string(std::chrono::duration<double>(imu2.timestamp.time_since_epoch()).count()) + ".");
    }

    // Special case
    if (imu1.timestamp == imu2.timestamp) {
        return imu1;
    }

    // Interpolation constant
    const double interp = std::chrono::duration<double>(interp_timestamp - imu1.timestamp).count() /
                          std::chrono::duration<double>(imu2.timestamp - imu1.timestamp).count();

    // Return interpolated Imu measurement.
    return Imu{imu1.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0)) ||
                               imu2.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0))
                       ? Eigen::Quaterniond(0, 0, 0, 0)
                       : imu1.orientation.slerp(interp, imu2.orientation),
            math::lerp(imu1.angular_velocity, imu2.angular_velocity, interp),
            math::lerp(imu1.linear_acceleration, imu2.linear_acceleration, interp),
            math::lerp(imu1.orientation_covariance, imu2.orientation_covariance, interp),
            math::lerp(imu1.angular_velocity_covariance, imu2.angular_velocity_covariance, interp),
            math::lerp(imu1.linear_acceleration_covariance, imu2.linear_acceleration_covariance, interp),
            interp_timestamp, interp <= 0.5 ? imu1.frame : imu2.frame};
}

}
