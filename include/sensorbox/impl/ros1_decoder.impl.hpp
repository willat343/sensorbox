#ifndef SENSORBOX_IMPL_ROS1_DECODER_IMPL_HPP
#define SENSORBOX_IMPL_ROS1_DECODER_IMPL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cppbox/exceptions.hpp>

#include "sensorbox/impl/ros1_decoder.hpp"
#include "sensorbox/impl/sensorbox.hpp"

namespace sensorbox {

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::chrono::nanoseconds& out) {
    // Duration is sec/nsecs as int32_t
    const std::chrono::seconds secs{read<int32_t>()};
    const std::chrono::nanoseconds nsecs{read<int32_t>()};
    out = secs + nsecs;
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::chrono::steady_clock::time_point& out) {
    // Time is sec/nsecs as uint32_t
    const std::chrono::seconds secs{read<uint32_t>()};
    const std::chrono::nanoseconds nsecs{read<uint32_t>()};
    out = std::chrono::steady_clock::time_point{secs + nsecs};
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::chrono::system_clock::time_point& out) {
    // Time is sec/nsecs as uint32_t
    const std::chrono::seconds secs{read<uint32_t>()};
    const std::chrono::nanoseconds nsecs{read<uint32_t>()};
    out = std::chrono::system_clock::time_point{secs + nsecs};
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(Eigen::Ref<Eigen::Vector3d> out) {
    out[0] = read<double>();
    out[1] = read<double>();
    out[2] = read<double>();
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(Eigen::Quaterniond& out) {
    out.x() = read<double>();
    out.y() = read<double>();
    out.z() = read<double>();
    out.w() = read<double>();
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(Eigen::Isometry3d& out) {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    if (msg_type() == "geometry_msgs/Pose") {
        decode_internal_to("geometry_msgs/Point", t);
        decode_internal_to("geometry_msgs/Quaternion", q);
    } else if (msg_type() == "geometry_msgs/PoseWithCovariance") {
        decode_internal_to("geometry_msgs/Point", t);
        decode_internal_to("geometry_msgs/Quaternion", q);
        ignore<double>(36);  // covariance
    } else if (msg_type() == "geometry_msgs/Transform") {
        decode_internal_to("geometry_msgs/Vector3", t);
        decode_internal_to("geometry_msgs/Quaternion", q);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to Eigen::Isometry3d.");
    }
    out = Eigen::Isometry3d::TranslationType{t} * q;
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(ActuatorMeasurement& out) {
    if (msg_type() == "series_elastic_actuator_msgs/SeActuatorReading") {
        ignore("std_msgs/Header");
        decode_internal_to("series_elastic_actuator_msgs/SeActuatorState", out);
        ignore("series_elastic_actuator_msgs/SeActuatorCommand");
    } else if (msg_type() == "series_elastic_actuator_msgs/SeActuatorState") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.name());
        out.set_type(ActuatorType::SERIES_ELASTIC);
        ignore<uint32_t>();  // statusword
        read_to_optional<double>(out.current());
        read_to_optional<double>(out.motor_position());
        read_to_optional<double>(out.motor_velocity());
        read_to_optional<double>(out.joint_position());
        read_to_optional<double>(out.joint_velocity());
        ignore<double>();
        read_to_optional(out.joint_torque());
        ignore("sensor_msgs/Imu");
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ActuatorMeasurement.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::vector<ActuatorMeasurement>& out) {
    if (msg_type() == "series_elastic_actuator_msgs/SeActuatorReadings") {
        decode_vector_to("series_elastic_actuator_msgs/SeActuatorReading", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<ActuatorMeasurement>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(ContactClassifications& out) {
    if (msg_type() == "anymal_msgs/AnymalState") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalMeasurement&>(out));
        ignore<int8_t>();                                 // state
        ignore("geometry_msgs/PoseStamped");              // pose
        ignore("geometry_msgs/TwistStamped");             // twist
        ignore("any_msgs/ExtendedJointState");            // joints
        const uint32_t contacts_size = read<uint32_t>();  // contacts size
        for (uint32_t i = 0; i < contacts_size; ++i) {
            ignore("std_msgs/Header");  // header
            const std::string name = read<std::string>();
            const uint8_t state = read<uint8_t>();
            out.set_classication(name, state == 1);
            ignore("geometry_msgs/Wrench");   // wrench
            ignore("geometry_msgs/Point");    // position
            ignore("geometry_msgs/Vector3");  // normal
            ignore<double>();                 // frictionCoefficient
            ignore<double>();                 // restitutionCoefficient
        }
        ignore_vector("geometry_msgs/TransformStamped");  // frame_transforms
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ContactClassifications.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(ImuMeasurement<3>& out) {
    if (msg_type() == "sensor_msgs/Imu") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        ignore("geometry_msgs/Quaternion");  // orientation
        ignore<double>(9);                   // orientation_covariance
        decode_internal_to("geometry_msgs/Vector3", out.angular_velocity());
        ignore<double>(9);  // orientation_covariance
        decode_internal_to("geometry_msgs/Vector3", out.linear_acceleration());
        ignore<double>(9);  // orientation_covariance
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ImuMeasurement<3>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(PoseMeasurement<3>& out) {
    if (msg_type() == "geometry_msgs/PoseStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/Pose", out.pose());
    } else if (msg_type() == "geometry_msgs/PoseWithCovarianceStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/PoseWithCovariance", out.pose());
    } else if (msg_type() == "geometry_msgs/TransformStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        decode_internal_to("geometry_msgs/Transform", out.pose());
    } else if (msg_type() == "nav_msgs/Odometry") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        decode_internal_to("geometry_msgs/PoseWithCovariance", out.pose());
        ignore("geometry_msgs/TwistWithCovariance");  // twist
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to Pose<3>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::vector<PoseMeasurement<3>>& out) {
    if (msg_type() == "tf2_msgs/TFMessage") {
        decode_vector_to("geometry_msgs/TransformStamped", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<Pose<3>>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(TemporalMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        ignore<uint32_t>();  // seq
        read_to(out.timestamp());
        ignore_string();  // frame_id
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalMeasurement.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(TemporalSpatialMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        ignore<uint32_t>();  // seq
        read_to(out.timestamp());
        read_to(out.frame());
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalSpatialMeasurement.");
    }
}

SENSORBOX_INLINE ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_, ROS1BytesDecoder* parent_decoder_)
    : cppbox::BytesDecoder(bytes_, size_, parent_decoder_), msg_type_(msg_type_) {}

SENSORBOX_INLINE std::size_t ROS1BytesDecoder::internal_msg_size(const std::string_view internal_msg_type,
        std::size_t offset) const {
    const std::size_t initial_offset = offset;
    if (internal_msg_type == "duration") {
        offset += sizeof(int32_t) * 2;  // secs, nsecs
    } else if (internal_msg_type == "string") {
        // In ROS 1, the string length in chars/bytes is encoded in the first 4 bytes as a uint32
        offset += sizeof(uint32_t) + peak<uint32_t>(offset);
    } else if (internal_msg_type == "time") {
        offset += sizeof(uint32_t) * 2;  // secs, nsecs
    } else if (internal_msg_type == "std_msgs/Duration") {
        offset += internal_msg_size("duration", offset);
    } else if (internal_msg_type == "std_msgs/Header") {
        offset += sizeof(uint32_t);  // seq
        offset += internal_msg_size("time", offset);
        offset += internal_msg_size("string", offset);
    } else if (internal_msg_type == "std_msgs/String") {
        offset += internal_msg_size("string", offset);
    } else if (internal_msg_type == "std_msgs/Time") {
        offset += internal_msg_size("time", offset);
    } else if (internal_msg_type == "geometry_msgs/Point") {
        offset += sizeof(double) * 3;  // x, y, z
    } else if (internal_msg_type == "geometry_msgs/Pose") {
        offset += internal_msg_size("geometry_msgs/Point", offset);
        offset += internal_msg_size("geometry_msgs/Quaternion", offset);
    } else if (internal_msg_type == "geometry_msgs/PoseStamped") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("geometry_msgs/Pose", offset);
    } else if (internal_msg_type == "geometry_msgs/PoseWithCovariance") {
        offset += internal_msg_size("geometry_msgs/Pose", offset);
        offset += sizeof(double) * 36;
    } else if (internal_msg_type == "geometry_msgs/PoseWithCovarianceStamped") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("geometry_msgs/PoseWithCovariance", offset);
    } else if (internal_msg_type == "geometry_msgs/Transform") {
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += internal_msg_size("geometry_msgs/Quaternion", offset);
    } else if (internal_msg_type == "geometry_msgs/Quaternion") {
        offset += sizeof(double) * 4;  // x, y, z, w
    } else if (internal_msg_type == "geometry_msgs/TransformStamped") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += internal_msg_size("geometry_msgs/Transform", offset);
    } else if (internal_msg_type == "geometry_msgs/Twist") {
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
    } else if (internal_msg_type == "geometry_msgs/TwistStamped") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("geometry_msgs/Twist", offset);
    } else if (internal_msg_type == "geometry_msgs/TwistWithCovariance") {
        offset += internal_msg_size("geometry_msgs/Twist", offset);
        offset += sizeof(double) * 36;
    } else if (internal_msg_type == "geometry_msgs/TwistWithCovarianceStamped") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("geometry_msgs/TwistWithCovariance", offset);
    } else if (internal_msg_type == "geometry_msgs/Vector3") {
        offset += sizeof(double) * 3;  // x, y, z
    } else if (internal_msg_type == "geometry_msgs/Wrench") {
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
    } else if (internal_msg_type == "nav_msgs/Odometry") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += internal_msg_size("geometry_msgs/PoseWithCovariance", offset);
        offset += internal_msg_size("geometry_msgs/TwistWithCovariance", offset);
    } else if (internal_msg_type == "sensor_msgs/Imu") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("geometry_msgs/Quaternion", offset);
        offset += sizeof(double) * 9;
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += sizeof(double) * 9;
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += sizeof(double) * 9;
    } else if (internal_msg_type == "tf2_msgs/TFMessage") {
        offset += internal_vector_msg_size("geometry_msgs/TransformStamped", offset);
    } else if (internal_msg_type == "any_msgs/ExtendedJointState") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_vector_msg_size("string", offset);
        offset += internal_vector_msg_size<double>(offset);
        offset += internal_vector_msg_size<double>(offset);
        offset += internal_vector_msg_size<double>(offset);
        offset += internal_vector_msg_size<double>(offset);
    } else if (internal_msg_type == "anymal_msgs/AnymalState") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += sizeof(int8_t);
        offset += internal_msg_size("geometry_msgs/PoseStamped", offset);
        offset += internal_msg_size("geometry_msgs/TwistStamped", offset);
        offset += internal_msg_size("any_msgs/ExtendedJointState", offset);
        offset += internal_vector_msg_size("anymal_msgs/Contact", offset);
        offset += internal_vector_msg_size("geometry_msgs/TransformStamped", offset);
    } else if (internal_msg_type == "anymal_msgs/Contact") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += sizeof(uint8_t);
        offset += internal_msg_size("geometry_msgs/Wrench", offset);
        offset += internal_msg_size("geometry_msgs/Point", offset);
        offset += internal_msg_size("geometry_msgs/Vector3", offset);
        offset += sizeof(double);
        offset += sizeof(double);
    } else if (internal_msg_type == "series_elastic_actuator_msgs/SeActuatorCommand") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += sizeof(int16_t);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(float);
        offset += sizeof(float);
        offset += sizeof(float);
    } else if (internal_msg_type == "series_elastic_actuator_msgs/SeActuatorReadings") {
        offset += internal_vector_msg_size("series_elastic_actuator_msgs/SeActuatorReading", offset);
    } else if (internal_msg_type == "series_elastic_actuator_msgs/SeActuatorReading") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("series_elastic_actuator_msgs/SeActuatorState", offset);
        offset += internal_msg_size("series_elastic_actuator_msgs/SeActuatorCommand", offset);
    } else if (internal_msg_type == "series_elastic_actuator_msgs/SeActuatorState") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += sizeof(uint32_t);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += sizeof(double);
        offset += internal_msg_size("sensor_msgs/Imu", offset);
    } else {
        throw_here("ROS1 internal msg size for msg_type " + std::string(internal_msg_type) + " not known.");
    }
    return offset - initial_offset;
}

SENSORBOX_INLINE std::size_t ROS1BytesDecoder::internal_vector_msg_size(const std::string_view internal_msg_type,
        std::size_t offset) const {
    const std::size_t initial_offset = offset;
    // In ROS 1, the vector length in elements is encoded in the first 4 bytes as a uint32
    const uint32_t size = peak<uint32_t>(offset);
    offset += sizeof(uint32_t);
    for (uint32_t i = 0; i < size; ++i) {
        offset += internal_msg_size(internal_msg_type, offset);
    }
    return offset - initial_offset;
}

}

#endif
