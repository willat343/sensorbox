#ifndef SENSOBOX_IMPL_ROS1_DECODER_HPP
#define SENSOBOX_IMPL_ROS1_DECODER_HPP

#include <cppbox/exceptions.hpp>
#include <set>

#include "sensorbox/ros1_decoder.hpp"

namespace sensorbox {

inline ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_)
    : ROS1BytesDecoder(bytes_, size_, msg_type_, nullptr) {}

template<typename T>
inline T ROS1BytesDecoder::decode_to() {
    T out;
    decode_to<T>(out);
    return out;
}

template<typename T>
inline void ROS1BytesDecoder::decode_to(T& out) {
    if constexpr (is_decodable<T>()) {
        if (is_directly_decodable_to<T>()) {
            read_to(out);
        } else if (is_start_decodable_to<T>()) {
            create_internal_decoder(std::string(this->starts_with())).template decode_to<T>(out);
            ignore_remaining();
        } else {
            throw_here("Decoding failed because T cannot be decoded to.");
        }
        assert(is_finished());
    } else {
        throw_here("Decoding failed because T is not decodable.");
    }
}

template<typename T>
inline void ROS1BytesDecoder::decode_to_optional(std::optional<T>& out) {
    out = decode_to<T>();
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline void ROS1BytesDecoder::ignore(const std::size_t num_ignore) {
    cppbox::BytesDecoder::ignore<T>(num_ignore);
}

inline void ROS1BytesDecoder::ignore_string() {
    ignore("string");
}

template<typename T>
    requires(std::is_same_v<T, std::string>)
inline void ROS1BytesDecoder::ignore(const std::size_t num_ignore) {
    // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
    cppbox::BytesDecoder::ignore<std::string>(cppbox::BytesDecoder::read<uint32_t>());
}

inline void ROS1BytesDecoder::ignore(const std::string_view msg_type) {
    increment_offset(internal_msg_size(msg_type, 0));
}

inline void ROS1BytesDecoder::ignore_vector(const std::string_view msg_type) {
    increment_offset(internal_vector_msg_size(msg_type, 0));
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline void ROS1BytesDecoder::ignore_vector() {
    increment_offset(internal_vector_msg_size<T>(0));
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_decodable() {
    return !ROS1DecodabilityTraits<T>::msg_types.empty();
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_decodable_to(const std::string_view msg_type) {
    return is_directly_decodable_to<T>(msg_type) || is_start_decodable_to<T>(msg_type);
}

template<typename T>
inline bool ROS1BytesDecoder::is_decodable_to() const {
    return is_decodable_to<T>(this->msg_type());
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_directly_decodable_to(const std::string_view msg_type) {
    constexpr auto& decodable_msg_types = ROS1DecodabilityTraits<T>::msg_types;
    return std::find(decodable_msg_types.cbegin(), decodable_msg_types.cend(), msg_type) != decodable_msg_types.cend();
}

template<typename T>
inline bool ROS1BytesDecoder::is_directly_decodable_to() const {
    return is_directly_decodable_to<T>(this->msg_type());
}

template<typename T>
inline constexpr bool ROS1BytesDecoder::is_start_decodable_to(const std::string_view msg_type) {
    return is_directly_decodable_to<T>(starts_with(msg_type));
}

template<typename T>
inline bool ROS1BytesDecoder::is_start_decodable_to() const {
    return is_start_decodable_to<T>(this->msg_type());
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

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline void ROS1BytesDecoder::read_to(T& out) {
    out = read<T>();
}

inline void ROS1BytesDecoder::read_to(std::string& out) {
    out = read<std::string>();
}

template<typename T>
inline void ROS1BytesDecoder::read_to_optional(std::optional<T>& out) {
    out = read_to<T>();
}

inline std::string_view ROS1BytesDecoder::starts_with() {
    return starts_with(msg_type());
}

inline constexpr bool ROS1BytesDecoder::starts_with(const std::string_view msg_type,
        const std::string_view start_msg_type) {
    return starts_with(msg_type) == start_msg_type;
}

inline ROS1BytesDecoder ROS1BytesDecoder::create_internal_decoder(const std::string& internal_msg_type) {
    return ROS1BytesDecoder(bytes() + offset(), internal_msg_size(internal_msg_type, 0), internal_msg_type, this);
}

template<typename T>
inline void ROS1BytesDecoder::decode_internal_to(const std::string& internal_msg_type, T& out) {
    return create_internal_decoder(internal_msg_type).decode_to(out);
}

template<typename T>
inline void ROS1BytesDecoder::decode_vector_to(const std::string& vector_msg_type, std::vector<T>& out) {
    // ROS 1 dynamic-sized vectors store the number elements as a uint32_t in the first 4 bytes
    out.resize(read<uint32_t>());
    for (T& out_element : out) {
        decode_internal_to(vector_msg_type, out_element);
    }
}

template<typename T>
    requires(std::is_trivially_copyable_v<T>)
inline std::size_t ROS1BytesDecoder::internal_vector_msg_size(std::size_t offset) const {
    // In ROS 1, the vector length in elements is encoded in the first 4 bytes as a uint32
    return sizeof(uint32_t) + peak<uint32_t>(offset) * sizeof(T);
}

}

#if SENSORBOX_HEADER_ONLY
#include "sensorbox/impl/ros1_decoder.impl.hpp"
#endif

#endif
