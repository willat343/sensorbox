#ifndef SENSORBOX_IMPL_ROS1_DECODER_IMPL_HPP
#define SENSORBOX_IMPL_ROS1_DECODER_IMPL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cppbox/exceptions.hpp>

#include "sensorbox/impl/ros1_decoder.hpp"
#include "sensorbox/impl/sensorbox.hpp"

namespace sensorbox {

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(Eigen::Ref<Eigen::Vector3d> out) {
    // geometry_msgs/Point, geometry_msgs/Vector3
    out[0] = read<double>();
    out[1] = read<double>();
    out[2] = read<double>();
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(Eigen::Quaterniond& out) {
    // geometry_msgs/Quaternion
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
        ignore("std_msgs/Header");  // header
        decode_internal_to("series_elastic_actuator_msgs/SeActuatorState", out);
        ignore("series_elastic_actuator_msgs/SeActuatorCommand");  // command
    } else if (msg_type() == "series_elastic_actuator_msgs/SeActuatorState") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalSpatialMeasurement&>(out));
        read_to(out.name());
        out.set_type(ActuatorType::SERIES_ELASTIC);
        ignore<uint32_t>();  // statusword
        read_optional_to<double>(out.current());
        read_optional_to<double>(out.motor_position());
        read_optional_to<double>(out.motor_velocity());
        read_optional_to<double>(out.joint_position());
        read_optional_to<double>(out.joint_velocity());
        ignore<double>();  // joint_acceleration
        read_optional_to(out.joint_torque());
        ignore("sensor_msgs/Imu");  // imu
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ActuatorMeasurement.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::vector<ActuatorMeasurement>& out) {
    if (msg_type() == "anymal_msgs/AnymalState") {
        ignore("std_msgs/Header");             // header
        ignore<int8_t>();                      // state
        ignore("geometry_msgs/PoseStamped");   // pose
        ignore("geometry_msgs/TwistStamped");  // twist
        decode_internal_to("any_msgs/ExtendedJointState", out);
        ignore("anymal_msgs/Contact[]");             // contacts
        ignore("geometry_msgs/TransformStamped[]");  // frame_transforms
    } else if (msg_type() == "any_msgs/ExtendedJointState") {
        ignore<uint32_t>();  // seq
        const typename ActuatorMeasurement::Timestamp timestamp = read<typename ActuatorMeasurement::Timestamp>();
        const std::string frame = read<std::string>();
        const uint32_t names_size = read<uint32_t>();
        out.resize(names_size);
        for (uint32_t i = 0; i < names_size; ++i) {
            out[i].timestamp() = timestamp;
            out[i].frame() = frame;
            read_to(out[i].name());
        }
        const uint32_t position_size = read<uint32_t>();
        throw_if(position_size != position_size, "Size mismatch in any_msgs/ExtendedJointState arrays.");
        for (uint32_t i = 0; i < position_size; ++i) {
            read_optional_to<double>(out[i].joint_position());
        }
        const uint32_t velocity_size = read<uint32_t>();
        throw_if(velocity_size != velocity_size, "Size mismatch in any_msgs/ExtendedJointState arrays.");
        for (uint32_t i = 0; i < velocity_size; ++i) {
            read_optional_to<double>(out[i].joint_velocity());
        }
        ignore("float64[]");  // acceleration
        const uint32_t effort_size = read<uint32_t>();
        throw_if(effort_size != effort_size, "Size mismatch in any_msgs/ExtendedJointState arrays.");
        for (uint32_t i = 0; i < effort_size; ++i) {
            read_optional_to<double>(out[i].joint_torque());
        }
    } else if (msg_type() == "series_elastic_actuator_msgs/SeActuatorReadings") {
        decode_vector_to("series_elastic_actuator_msgs/SeActuatorReading", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<ActuatorMeasurement>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(ActuatorMeasurements& out) {
    if (msg_type() == "anymal_msgs/AnymalState") {
        ignore("std_msgs/Header");             // header
        ignore<int8_t>();                      // state
        ignore("geometry_msgs/PoseStamped");   // pose
        ignore("geometry_msgs/TwistStamped");  // twist
        decode_internal_to("any_msgs/ExtendedJointState", out);
        ignore("anymal_msgs/Contact[]");             // contacts
        ignore("geometry_msgs/TransformStamped[]");  // frame_transforms
    } else if (msg_type() == "any_msgs/ExtendedJointState") {
        out.timestamp() = peak<typename ActuatorMeasurements::Timestamp>(sizeof(uint32_t));  // (peak past header.seq)
        read_to(out.measurements());
    } else if (msg_type() == "series_elastic_actuator_msgs/SeActuatorReadings") {
        read_to(out.measurements());
        out.timestamp() =
                out.measurements().empty() ? ActuatorMeasurements::Timestamp() : out.measurements().front().timestamp();
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to ActuatorMeasurements.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(ContactClassifications& out) {
    if (msg_type() == "anymal_msgs/AnymalState") {
        decode_internal_to("std_msgs/Header", static_cast<TemporalMeasurement&>(out));
        ignore<int8_t>();                       // state
        ignore("geometry_msgs/PoseStamped");    // pose
        ignore("geometry_msgs/TwistStamped");   // twist
        ignore("any_msgs/ExtendedJointState");  // joints
        decode_internal_to("anymal_msgs/Contact[]", out);
        ignore("geometry_msgs/TransformStamped[]");  // frame_transforms
    } else if (msg_type() == "anymal_msgs/Contact[]") {
        const uint32_t contacts_size = read<uint32_t>();
        for (uint32_t i = 0; i < contacts_size; ++i) {
            decode_internal_to("std_msgs/Header", static_cast<TemporalMeasurement&>(out));  // header
            const std::string name = read<std::string>();
            const uint8_t state = read<uint8_t>();
            out.set_classication(name, state == 1);
            ignore("geometry_msgs/Wrench");   // wrench
            ignore("geometry_msgs/Point");    // position
            ignore("geometry_msgs/Vector3");  // normal
            ignore<double>();                 // frictionCoefficient
            ignore<double>();                 // restitutionCoefficient
        }
    } else if (msg_type() == "anymal_msgs/Contacts") {
        decode_internal_to("anymal_msgs/Contact[]", out);
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
        ignore<double>(9);  // angular_velocity_covariance
        decode_internal_to("geometry_msgs/Vector3", out.linear_acceleration());
        ignore<double>(9);  // linear_acceleration_covariance
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
        throw_here("msg_type " + msg_type() + " cannot be converted to PoseMeasurement<3>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(std::vector<PoseMeasurement<3>>& out) {
    if (msg_type() == "nav_msgs/Path") {
        ignore("std_msgs/Header");  // header
        decode_vector_to("geometry_msgs/PoseStamped", out);
    } else if (msg_type() == "tf2_msgs/TFMessage") {
        decode_vector_to("geometry_msgs/TransformStamped", out);
    } else {
        throw_here("msg_type " + msg_type() + " cannot be converted to std::vector<PoseMeasurement<3>>.");
    }
}

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(PoseMeasurements<3>& out) {
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

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(PositionMeasurement<3>& out) {
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

SENSORBOX_INLINE void ROS1BytesDecoder::read_to(TemporalMeasurement& out) {
    if (msg_type() == "std_msgs/Header") {
        ignore<uint32_t>();  // seq
        read_to(out.timestamp());
        ignore("string");  // frame_id
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

SENSORBOX_INLINE std::size_t ROS1BytesDecoder::internal_msg_size(const std::string_view internal_msg_type,
        std::size_t extra_offset) {
    const std::size_t initial_offset = extra_offset;
    const std::size_t fundamental_size = ROS1MessagesTypes::fundamental::size(internal_msg_type);
    if (fundamental_size > 0) {
        // Message type is fundamental
        extra_offset += fundamental_size;
    } else if (message_is_vector_type(internal_msg_type)) {
        const std::string_view internal_msg_element_type = message_vector_type(internal_msg_type);
        // In ROS 1, the vector length in elements is encoded in the first 4 bytes as a uint32.
        const uint32_t vector_size = peak<uint32_t>(extra_offset);
        extra_offset += ROS1MessagesTypes::fundamental::size("uint32");
        for (uint32_t i = 0; i < vector_size; ++i) {
            extra_offset += internal_msg_size(internal_msg_element_type, extra_offset);
        }
    } else if (message_is_array_type(internal_msg_type)) {
        const std::string_view internal_msg_element_type = message_array_type(internal_msg_type);
        const std::size_t array_size = message_array_size(internal_msg_type);
        for (std::size_t i = 0; i < array_size; ++i) {
            extra_offset += internal_msg_size(internal_msg_element_type, extra_offset);
        }
    } else if (internal_msg_type == "string") {
        // In ROS 1, the string length in chars/bytes is encoded in the first 4 bytes as a uint32.
        extra_offset += ROS1MessagesTypes::fundamental::size("uint32") + peak<uint32_t>(extra_offset);
    } else {
        // Message type belongs to a group
        const auto fields = message_fields(ROS1MessagesTypes::msg_types, internal_msg_type);
        throw_if(fields.empty(), std::string(internal_msg_type) + " is not a known msg type.");
        std::for_each(fields.begin(), fields.end(),
                [this, &extra_offset, internal_msg_type](
                        const MessageField& field) { extra_offset += internal_msg_size(field.type, extra_offset); });
    }
    return extra_offset - initial_offset;
}

}

#endif
