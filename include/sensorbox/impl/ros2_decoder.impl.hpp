#ifndef SENSORBOX_IMPL_ROS2_DECODER_IMPL_HPP
#define SENSORBOX_IMPL_ROS2_DECODER_IMPL_HPP

#include "sensorbox/impl/ros2_decoder.hpp"
#include "sensorbox/impl/sensorbox.hpp"

namespace sensorbox {

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(Eigen::Ref<Eigen::Vector3d> out) {
    // geometry_msgs/Point, geometry_msgs/Vector3
    out[0] = read<double>();
    out[1] = read<double>();
    out[2] = read<double>();
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(Eigen::Quaterniond& out) {
    // geometry_msgs/Quaternion
    out.x() = read<double>();
    out.y() = read<double>();
    out.z() = read<double>();
    out.w() = read<double>();
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(Eigen::Isometry3d& out) {
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

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(ImuMeasurement<3>& out) {
    if (msg_type() == "sensor_msgs/Imu") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        ignore("geometry_msgs/Quaternion");  // orientation
        ignore<double>(9);                   // orientation_covariance
        decode_internal_to("geometry_msgs/Vector3", out.angular_velocity());
        ignore<double>(9);  // angular_velocity_covariance
        decode_internal_to("geometry_msgs/Vector3", out.linear_acceleration());
        ignore<double>(9);  // linear_acceleration_covariance
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ImuMeasurement<3>.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(PoseMeasurement<3>& out) {
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
        throw_here("msg_type " + msg_type() + " cannot be converted to PoseMeasurement<3>.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(std::vector<PoseMeasurement<3>>& out) {
    if (msg_type() == "nav_msgs/Path") {
        ignore("std_msgs/Header");  // header
        decode_vector_to("geometry_msgs/PoseStamped", out);
    } else if (msg_type() == "tf2_msgs/TFMessage") {
        decode_vector_to("geometry_msgs/TransformStamped", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<PoseMeasurement<3>>.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(PoseMeasurements<3>& out) {
    if (msg_type() == "nav_msgs/Path") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalMeasurement&>(out));
        decode_vector_to("geometry_msgs/PoseStamped", out.measurements());
    } else if (msg_type() == "tf2_msgs/TFMessage") {
        decode_vector_to("geometry_msgs/TransformStamped", out.measurements());
        out.timestamp() =
                out.measurements().empty() ? PoseMeasurements<3>::Timestamp() : out.measurements().front().timestamp();
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<PoseMeasurement<3>>.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(PositionMeasurement<3>& out) {
    if (msg_type() == "geometry_msgs/PointStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/Point", out.position());
    } else if (msg_type() == "geometry_msgs/Vector3Stamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/Vector3", out.position());
    } else if (msg_type() == "geometry_msgs/PoseStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/Pose", out.position());
    } else if (msg_type() == "geometry_msgs/PoseWithCovarianceStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        decode_internal_to("geometry_msgs/PoseWithCovariance", out.position());
    } else if (msg_type() == "geometry_msgs/TransformStamped") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        decode_internal_to("geometry_msgs/Transform", out.position());
    } else if (msg_type() == "nav_msgs/Odometry") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.child_frame());
        decode_internal_to("geometry_msgs/PoseWithCovariance", out.position());
        ignore("geometry_msgs/TwistWithCovariance");  // twist
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to PositionMeasurement<3>.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(TemporalMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        read_to(out.timestamp());
        ignore("string");  // frame_id
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalMeasurement.");
    }
}

SENSORBOX_INLINE void ROS2BytesDecoder::read_to(TemporalSpatialMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        read_to(out.timestamp());
        read_to(out.frame());
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to TemporalSpatialMeasurement.");
    }
}

SENSORBOX_INLINE std::size_t ROS2BytesDecoder::internal_msg_size(const std::string_view internal_msg_type,
        std::size_t offset_) const {
    // Note that offset does not include the CDR header
    const std::size_t initial_offset = offset_;
    const std::size_t fundamental_size = ROS2MessagesTypes::fundamental::size(internal_msg_type);
    if (fundamental_size > 0) {
        // Message type is fundamental
        offset_ += fundamental_size;
    } else if (message_is_vector_type(internal_msg_type)) {
        const std::string_view internal_msg_element_type = message_vector_type(internal_msg_type);
        // In ROS 2, the vector length in elements is encoded in the first 4 bytes as a uint32 (which must
        // obey alignment). Add padding first to ensure correct peak.
        offset_ += ROS2MessagesTypes::fundamental::padding(ROS2MessagesTypes::fundamental::size("uint32"), offset_);
        const uint32_t vector_size = peak<uint32_t>(offset_);
        offset_ += ROS2MessagesTypes::fundamental::size("uint32", offset_);
        for (uint32_t i = 0; i < vector_size; ++i) {
            offset_ += internal_msg_size(internal_msg_element_type, offset_);
        }
    } else if (message_is_array_type(internal_msg_type)) {
        const std::string_view internal_msg_element_type = message_array_type(internal_msg_type);
        const std::size_t array_size = message_array_size(internal_msg_type);
        for (std::size_t i = 0; i < array_size; ++i) {
            offset_ += internal_msg_size(internal_msg_element_type, offset_);
        }
    } else if (internal_msg_type == "string") {
        // In ROS 2, the string length in chars/bytes is encoded in the first 4 bytes as a uint32 (which
        // must obey alignment), and the string ends with a null terminator '\0'. Add padding first to
        // ensure correct peak.
        offset_ += ROS2MessagesTypes::fundamental::padding(ROS2MessagesTypes::fundamental::size("uint32"), offset_);
        offset_ += ROS2MessagesTypes::fundamental::size("uint32", offset_) + peak<uint32_t>(offset_) + 1;
    } else if (internal_msg_type.starts_with("builtin_interfaces")) {
        // In ROS 2, all builtin_interfaces has fixed size
        offset_ += ROS2MessagesTypes::builtin_interfaces::size(internal_msg_type, offset_);
    } else {
        // Message type belongs to a group
        const auto fields = message_fields(ROS2MessagesTypes::msg_types, internal_msg_type);
        throw_if(fields.empty(), std::string(internal_msg_type) + " is not a known msg type.");
        std::for_each(fields.begin(), fields.end(), [this, &offset_, internal_msg_type](const MessageField& field) {
            offset_ += internal_msg_size(field.type, offset_);
        });
    }
    return offset_ - initial_offset;
}

}

#endif
