#ifndef SENSOBOX_IMPL_ROS1_DECODER_HPP
#define SENSOBOX_IMPL_ROS1_DECODER_HPP

#include <cassert>
#include <cppbox/exceptions.hpp>
#include <set>

#include "sensorbox/ros1_decoder.hpp"

namespace sensorbox {

inline ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_)
    : ROS1BytesDecoder(bytes_, size_, msg_type_, nullptr) {
    assert(internal_msg_size(msg_type()) == size());
}

template<typename T>
inline T ROS1BytesDecoder::decode() {
    T out;
    decode_to<T>(out);
    return out;
}

template<typename T>
inline void ROS1BytesDecoder::decode_to(T& out) {
    if (is_directly_decodable_to<T>()) {
        read_to(out);
    } else if (is_vector_or_start_recursively_decodable_to<T>()) {
        std::string_view starts_with_ = starts_with();
        if (message_is_vector_type(starts_with_)) {
            throw_if(this->template read<uint32_t>() == 0, "Decoding of vector failed because vector was empty.");
        }
        create_internal_decoder(std::string(message_vector_type(starts_with_))).template decode_to<T>(out);
        ignore_remaining();
    } else {
        throw_here("Decoding failed because T cannot be decoded to, from message type " + msg_type() + ".");
    }
    assert(is_finished());
}

template<typename T>
inline void ROS1BytesDecoder::decode_optional_to(std::optional<T>& out) {
    out = decode_to<T>();
}

template<typename T>
inline std::optional<T> ROS1BytesDecoder::decode_optional() {
    std::optional<T> out;
    decode_optional_to<T>(out);
    return out;
}

inline void ROS1BytesDecoder::ignore(const std::string_view msg_type, const std::size_t num_ignore) {
    ignore_bytes(internal_msg_size(msg_type) * num_ignore);
}

template<typename T>
inline T ROS1BytesDecoder::peak(const std::size_t extra_offset) {
    const std::size_t initial_offset = offset();
    increment_offset(extra_offset);
    const T out = read<T>();
    decrement_offset(offset() - initial_offset);
    return out;
}

template<typename T>
inline T ROS1BytesDecoder::read() {
    T out;
    read_to(out);
    return out;
}

template<typename T>
inline std::optional<T> ROS1BytesDecoder::read_optional() {
    std::optional<T> out;
    read_optional_to<T>(out);
    return out;
}

template<typename T>
inline void ROS1BytesDecoder::read_optional_to(std::optional<T>& out) {
    out = read<T>();
}

template<typename T>
    requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
inline void ROS1BytesDecoder::read_to(T& out) {
    out = cppbox::BytesDecoder::read<T>();
}

inline void ROS1BytesDecoder::read_to(std::string& out) {
    // In ROS 1, the string length in bytes is encoded in the first 4 bytes as a uint32
    out = cppbox::BytesDecoder::read<std::string>(read<uint32_t>());
}

template<cppbox::IsDuration T>
inline void ROS1BytesDecoder::read_to(T& out) {
    // In ROS 1, duration is sec as int32_t and nsec as int32_t
    const std::chrono::seconds sec{read<int32_t>()};
    const std::chrono::nanoseconds nsec{read<int32_t>()};
    out = T{sec + nsec};
}

template<cppbox::IsTimePoint T>
inline void ROS1BytesDecoder::read_to(T& out) {
    // In ROS 1, time is sec as uint32_t and nsec as uint32_t
    const std::chrono::seconds sec{read<uint32_t>()};
    const std::chrono::nanoseconds nsec{read<uint32_t>()};
    out = T{sec + nsec};
}

inline ROS1BytesDecoder::ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_, ROS1BytesDecoder* parent_decoder_)
    : MessageDecoder<ROS1MessagesTypes, ROS1Conversions>(bytes_, size_, msg_type_, parent_decoder_) {}

inline ROS1BytesDecoder ROS1BytesDecoder::create_internal_decoder(const std::string& internal_msg_type) {
    const std::size_t internal_msg_size_ = internal_msg_size(internal_msg_type);
    assert(offset() + internal_msg_size_ <= size());
    return ROS1BytesDecoder(bytes() + offset(), internal_msg_size_, internal_msg_type, this);
}

template<typename T>
inline T ROS1BytesDecoder::decode_internal(const std::string& internal_msg_type) {
    T out;
    decode_internal_to(internal_msg_type, out);
    return out;
}

template<typename T>
inline void ROS1BytesDecoder::decode_internal_to(const std::string& internal_msg_type, T& out) {
    create_internal_decoder(internal_msg_type).decode_to(out);
}

template<typename T>
inline std::vector<T> ROS1BytesDecoder::decode_vector(const std::string& internal_msg_type) {
    std::vector<T> out;
    decode_vector_to(internal_msg_type, out);
    return out;
}

template<typename T>
inline void ROS1BytesDecoder::decode_vector_to(const std::string& vector_msg_type, std::vector<T>& out) {
    // In ROS 1, dynamic-sized vectors store the number elements as a uint32_t in the first 4 bytes
    out.resize(read<uint32_t>());
    for (T& out_element : out) {
        decode_internal_to(vector_msg_type, out_element);
    }
}

}

#if SENSORBOX_HEADER_ONLY
#include "sensorbox/impl/ros1_decoder.impl.hpp"
#endif

#endif
