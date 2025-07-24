#include "sensorbox/ros1_decoder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sensorbox {

ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_)
    : ROS1BytesDecoder(bytes_, size_, msg_type_, nullptr) {
    assert(size_ == internal_msg_size(msg_type(), 0));
}

ROS1BytesDecoder ROS1BytesDecoder::create_internal_decoder(const std::string& internal_msg_type) {
    return ROS1BytesDecoder(bytes() + offset(), internal_msg_size(internal_msg_type, 0), internal_msg_type, this);
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
        ignore<double>(36);  // covariance ignored
    } else if (msg_type() == "geometry_msgs/Transform") {
        create_internal_decoder("geometry_msgs/Vector3").read_to(t);
        create_internal_decoder("geometry_msgs/Quaternion").read_to(q);
    } else {
        throw std::runtime_error("msg_type " + msg_type() + " cannot be converted to Eigen::Isometry3d.");
    }
    out = Eigen::Isometry3d::TranslationType{t} * q;
}

void ROS1BytesDecoder::read_to(UnaryMeasurement& out) {
    ignore<uint32_t>();  // seq ignored
    read_to(out.timestamp());
    read_to(out.frame());
}

void ROS1BytesDecoder::read_to(PoseMeasurement<3>& out) {
    if (msg_type() == "geometry_msgs/PoseStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<UnaryMeasurement&>(out));
        create_internal_decoder("geometry_msgs/Pose").read_to(out.pose());
    } else if (msg_type() == "geometry_msgs/PoseWithCovarianceStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<UnaryMeasurement&>(out));
        create_internal_decoder("geometry_msgs/PoseWithCovariance").read_to(out.pose());
    } else if (msg_type() == "geometry_msgs/TransformStamped") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<UnaryMeasurement&>(out));
        read_to(out.child_frame());
        create_internal_decoder("geometry_msgs/Transform").read_to(out.pose());
    } else if (msg_type() == "nav_msgs/Odometry") {
        create_internal_decoder("std_msgs/Header").read_to(static_cast<UnaryMeasurement&>(out));
        read_to(out.child_frame());
        create_internal_decoder("geometry_msgs/PoseWithCovariance").read_to(out.pose());
        ignore("geometry_msgs/TwistWithCovariance");  // twist ignored
    } else {
        throw std::runtime_error("msg_type " + msg_type() + " cannot be converted to PoseMeasurement<3>.");
    }
}

ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
        ROS1BytesDecoder* parent_decoder_)
    : cppbox::BytesDecoder(bytes_, size_, parent_decoder_),
      msg_type_(msg_type_) {}

std::size_t ROS1BytesDecoder::internal_msg_size(const std::string& internal_msg_type, std::size_t offset) const {
    const std::size_t initial_offset = offset;
    if (internal_msg_type == "string") {
        // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
        offset += sizeof(uint32_t) + peak<uint32_t>(offset);
    } else if (internal_msg_type == "time") {
        offset += sizeof(uint32_t) * 2;
    } else if (internal_msg_type == "std_msgs/Header") {
        offset += sizeof(uint32_t);
        offset += internal_msg_size("time", offset);
        offset += internal_msg_size("string", offset);
    } else if (internal_msg_type == "geometry_msgs/Point") {
        offset += sizeof(double) * 3;
    } else if (internal_msg_type == "geometry_msgs/Vector3") {
        offset += sizeof(double) * 3;
    } else if (internal_msg_type == "geometry_msgs/Quaternion") {
        offset += sizeof(double) * 4;
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
    } else if (internal_msg_type == "nav_msgs/Odometry") {
        offset += internal_msg_size("std_msgs/Header", offset);
        offset += internal_msg_size("string", offset);
        offset += internal_msg_size("geometry_msgs/PoseWithCovariance", offset);
        offset += internal_msg_size("geometry_msgs/TwistWithCovariance", offset);
    } else {
        throw std::runtime_error("ROS1 internal msg size for msg_type " + internal_msg_type + " not known.");
    }
    return offset - initial_offset;
}

}
