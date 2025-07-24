#ifndef SENSOBOX_IMPL_ROS1_DECODER_HPP
#define SENSOBOX_IMPL_ROS1_DECODER_HPP

#include <set>

#include "sensorbox/ros1_decoder.hpp"

namespace sensorbox {

template<typename T>
inline T ROS1BytesDecoder::decode_to() {
    const T out = read_to<T>();
    assert(is_finished());
    return out;
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

inline void ROS1BytesDecoder::read_to(std::chrono::nanoseconds& out) {
    // Careful ordering of read functions
    const std::chrono::seconds sec{read<uint32_t>()};
    const std::chrono::nanoseconds nsec{read<uint32_t>()};
    out = sec + nsec;
}

inline void ROS1BytesDecoder::read_to(std::chrono::steady_clock::time_point& out) {
    out = std::chrono::steady_clock::time_point{read_to<std::chrono::nanoseconds>()};
}

inline void ROS1BytesDecoder::read_to(std::chrono::system_clock::time_point& out) {
    out = std::chrono::system_clock::time_point{read_to<std::chrono::nanoseconds>()};
}

}

#endif
