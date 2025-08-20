#include "sensorbox/ros1_decoder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cppbox/exceptions.hpp>

namespace sensorbox {

ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_)
    : ROS1BytesDecoder(bytes_, size_, msg_type_, nullptr) {}

ROS1BytesDecoder ROS1BytesDecoder::create_internal_decoder(const std::string& internal_msg_type) {
    return ROS1BytesDecoder(bytes() + offset(), internal_msg_size(internal_msg_type, 0), internal_msg_type, this);
}

void ROS1BytesDecoder::read_to(std::chrono::nanoseconds& out) {
    // Duration is sec/nsecs as int32_t
    const std::chrono::seconds secs{read<int32_t>()};
    const std::chrono::nanoseconds nsecs{read<int32_t>()};
    out = secs + nsecs;
}

void ROS1BytesDecoder::read_to(std::chrono::steady_clock::time_point& out) {
    // Time is sec/nsecs as uint32_t
    const std::chrono::seconds secs{read<uint32_t>()};
    const std::chrono::nanoseconds nsecs{read<uint32_t>()};
    out = std::chrono::steady_clock::time_point{secs + nsecs};
}

void ROS1BytesDecoder::read_to(std::chrono::system_clock::time_point& out) {
    // Time is sec/nsecs as uint32_t
    const std::chrono::seconds secs{read<uint32_t>()};
    const std::chrono::nanoseconds nsecs{read<uint32_t>()};
    out = std::chrono::system_clock::time_point{secs + nsecs};
}

void ROS1BytesDecoder::read_to(Eigen::Ref<Eigen::Vector3d> out) {
    out[0] = read<double>();
    out[1] = read<double>();
    out[2] = read<double>();
}

void ROS1BytesDecoder::read_to(Eigen::Quaterniond& out) {
    out.x() = read<double>();
    out.y() = read<double>();
    out.z() = read<double>();
    out.w() = read<double>();
}

void ROS1BytesDecoder::read_to(Eigen::Isometry3d& out) {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    if (msg_type() == "geometry_msgs/Pose") {
        create_internal_decoder("geometry_msgs/Point").read_to(t);
        create_internal_decoder("geometry_msgs/Quaternion").read_to(q);
    } else if (msg_type() == "geometry_msgs/PoseWithCovariance") {
        create_internal_decoder("geometry_msgs/Point").read_to(t);
        create_internal_decoder("geometry_msgs/Quaternion").read_to(q);
        ignore<double>(36);  // covariance
    } else if (msg_type() == "geometry_msgs/Transform") {
        create_internal_decoder("geometry_msgs/Vector3").read_to(t);
        create_internal_decoder("geometry_msgs/Quaternion").read_to(q);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to Eigen::Isometry3d.");
    }
    out = Eigen::Isometry3d::TranslationType{t} * q;
}

void ROS1BytesDecoder::read_to(ContactClassifications& out) {
    if (msg_type() == "anymal_msgs/AnymalState") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalMeasurement&>(out));
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

void ROS1BytesDecoder::read_to(ImuMeasurement<3>& out) {
    if (msg_type() == "sensor_msgs/Imu") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalSpatialMeasurement&>(out));
        ignore("geometry_msgs/Quaternion");  // orientation
        ignore<double>(9);                   // orientation_covariance
        create_internal_decoder("geometry_msgs/Vector3").read_to(out.angular_velocity());
        ignore<double>(9);  // orientation_covariance
        create_internal_decoder("geometry_msgs/Vector3").read_to(out.linear_acceleration());
        ignore<double>(9);  // orientation_covariance
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ImuMeasurement<3>.");
    }
}

void ROS1BytesDecoder::read_to(PoseMeasurement<3>& out) {
    if (msg_type() == "geometry_msgs/PoseStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalSpatialMeasurement&>(out));
        create_internal_decoder("geometry_msgs/Pose").read_to(out.pose());
    } else if (msg_type() == "geometry_msgs/PoseWithCovarianceStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalSpatialMeasurement&>(out));
        create_internal_decoder("geometry_msgs/PoseWithCovariance").read_to(out.pose());
    } else if (msg_type() == "geometry_msgs/TransformStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        create_internal_decoder("geometry_msgs/Transform").read_to(out.pose());
    } else if (msg_type() == "nav_msgs/Odometry") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        create_internal_decoder("geometry_msgs/PoseWithCovariance").read_to(out.pose());
        ignore("geometry_msgs/TwistWithCovariance");  // twist
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to PoseMeasurement<3>.");
    }
}

void ROS1BytesDecoder::read_to(TemporalMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        ignore<uint32_t>();  // seq
        read_to(out.timestamp());
        ignore_string();  // frame_id
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalMeasurement.");
    }
}

void ROS1BytesDecoder::read_to(TemporalSpatialMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        ignore<uint32_t>();  // seq
        read_to(out.timestamp());
        read_to(out.frame());
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalSpatialMeasurement.");
    }
}

void ROS1BytesDecoder::read_to(std::vector<PoseMeasurement<3>>& out) {
    if (msg_type() == "tf2_msgs/TFMessage") {
        read_vector_to("geometry_msgs/TransformStamped", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<PoseMeasurement<3>>.");
    }
}

ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
        ROS1BytesDecoder* parent_decoder_)
    : cppbox::BytesDecoder(bytes_, size_, parent_decoder_), msg_type_(msg_type_) {
    assert(is_decodable(msg_type_));
    assert(size_ == internal_msg_size(msg_type(), 0));
}

std::size_t ROS1BytesDecoder::internal_msg_size(const std::string& internal_msg_type, std::size_t offset) const {
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
    } else {
        throw_here("ROS1 internal msg size for msg_type " + internal_msg_type + " not known.");
    }
    return offset - initial_offset;
}

std::size_t ROS1BytesDecoder::internal_vector_msg_size(const std::string& internal_msg_type, std::size_t offset) const {
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
