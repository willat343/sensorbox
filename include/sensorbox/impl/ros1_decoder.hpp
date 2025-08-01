#ifndef SENSOBOX_IMPL_ROS1_DECODER_HPP
#define SENSOBOX_IMPL_ROS1_DECODER_HPP

#include <set>

#include "sensorbox/ros1_decoder.hpp"

namespace sensorbox {

template<typename T>
inline T ROS1BytesDecoder::decode_to() {
    if constexpr (is_decodable<T>()) {
        const T out = read_to<T>();
        assert(is_finished());
        return out;
    } else {
        throw std::runtime_error("ROS1BytesDecoder::decode_to<T>() failed because T cannot be decoded to.");
    }
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline void ROS1BytesDecoder::ignore(const std::size_t num_ignore) {
    cppbox::BytesDecoder::ignore<T>(num_ignore);
}

template<typename T>
    requires(std::is_same_v<T, std::string>)
inline void ROS1BytesDecoder::ignore(const std::size_t num_ignore) {
    // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
    cppbox::BytesDecoder::ignore<std::string>(cppbox::BytesDecoder::read<uint32_t>());
}

inline void ROS1BytesDecoder::ignore(const std::string& msg_type) {
    increment_offset(internal_msg_size(msg_type, 0));
}

inline constexpr bool ROS1BytesDecoder::is_decodable(const std::string_view& msg_type) {
    constexpr std::array<std::string_view, 23> decodable_msg_types{"duration", "string", "time", "std_msgs/Duration",
            "std_msgs/Header", "std_msgs/String", "std_msgs/Time", "geometry_msgs/Point", "geometry_msgs/Pose",
            "geometry_msgs/PoseStamped", "geometry_msgs/PoseWithCovariance", "geometry_msgs/PoseWithCovarianceStamped",
            "geometry_msgs/Quaternion", "geometry_msgs/Transform", "geometry_msgs/TransformStamped",
            "geometry_msgs/Twist", "geometry_msgs/TwistStamped", "geometry_msgs/TwistWithCovariance",
            "geometry_msgs/TwistWithCovarianceStamped", "geometry_msgs/Vector3", "nav_msgs/Odometry",
            "sensor_msgs/Imu", "tf2_msgs/TFMessage"};
    return std::find(decodable_msg_types.cbegin(), decodable_msg_types.cend(), msg_type) != decodable_msg_types.cend();
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_decodable() {
    return !ROS1DecodabilityTraits<T>::msg_types.empty();
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_decodable_to(const std::string_view& msg_type) {
    constexpr auto decodable_msg_types = ROS1DecodabilityTraits<T>::msg_types;
    return std::find(decodable_msg_types.cbegin(), decodable_msg_types.cend(), msg_type) != decodable_msg_types.cend();
}

template<typename T>
bool ROS1BytesDecoder::is_decodable_to() {
    return is_decodable_to<T>(msg_type());
}

inline const std::string& ROS1BytesDecoder::msg_type() const {
    return msg_type_;
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline T ROS1BytesDecoder::peak(const std::size_t extra_offset) const {
    return cppbox::BytesDecoder::peak<T>(extra_offset);
}

template<typename T>
    requires(std::is_same_v<T, std::string>)
inline T ROS1BytesDecoder::peak(const std::size_t extra_offset) const {
    // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
    return cppbox::BytesDecoder::peak<std::string>(cppbox::BytesDecoder::peak<uint32_t>(extra_offset),
            extra_offset + sizeof(uint32_t));
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline T ROS1BytesDecoder::read() {
    return cppbox::BytesDecoder::read<T>();
}

template<typename T>
    requires(std::is_same_v<T, std::string>)
inline T ROS1BytesDecoder::read() {
    // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
    return cppbox::BytesDecoder::read<std::string>(cppbox::BytesDecoder::read<uint32_t>());
}

template<typename T>
inline T ROS1BytesDecoder::read_to() {
    T out;
    read_to(out);
    return out;
}

inline void ROS1BytesDecoder::read_to(std::string& out) {
    out = read<std::string>();
}

}

#endif
