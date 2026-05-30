#ifndef SENSORBOX_IMPL_ROS2_DECODER_HPP
#define SENSORBOX_IMPL_ROS2_DECODER_HPP

#include "cppbox/exceptions.hpp"
#include "sensorbox/ros2_decoder.hpp"

namespace sensorbox {

inline ROS2BytesDecoder::ROS2BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_)
    : ROS2BytesDecoder(bytes_ + ROS2MessagesTypes::cdr_header.size(), size_ - ROS2MessagesTypes::cdr_header.size(),
              ROS2MessagesTypes::remove_internal_msg_substring(msg_type_), nullptr) {
    // Check that the CDR header was correctly ignored (and is not included in the offset)
    assert(size_ >= ROS2MessagesTypes::cdr_header.size());
    assert(std::equal(ROS2MessagesTypes::cdr_header.begin(), ROS2MessagesTypes::cdr_header.end(), bytes_));
}

template<typename T>
inline T ROS2BytesDecoder::decode() {
    T out;
    decode_to<T>(out);
    return out;
}

template<typename T>
inline void ROS2BytesDecoder::decode_to(T& out) {
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
inline void ROS2BytesDecoder::decode_optional_to(std::optional<T>& out) {
    out = decode<T>();
}

template<typename T>
inline std::optional<T> ROS2BytesDecoder::decode_optional() {
    std::optional<T> out;
    decode_optional_to<T>(out);
    return out;
}

inline void ROS2BytesDecoder::ignore(const std::string_view msg_type, const std::size_t num_ignore) {
    for (std::size_t i = 0; i < num_ignore; ++i) {
        ignore_bytes(internal_msg_size(msg_type, 0));
    }
}

template<typename T>
    requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
inline void ROS2BytesDecoder::ignore(const std::size_t num_ignore) {
    // In ROS 2, fundamental types may have padding which must be ignored
    ignore_bytes(ROS2MessagesTypes::fundamental::padding(sizeof(T), offset()));
    // Subsequently, types will be aligned without padding so can be be ignored together
    ignore_bytes(sizeof(T) * num_ignore);
}

template<typename T>
    requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
inline T ROS2BytesDecoder::peak(const std::size_t extra_offset) const {
    return cppbox::BytesDecoder::peak<T>(
            ROS2MessagesTypes::fundamental::padding(sizeof(T), offset() + extra_offset) + extra_offset);
}

template<typename T>
    requires(std::is_same_v<T, std::string>)
inline T ROS2BytesDecoder::peak(const std::size_t extra_offset) const {
    // In ROS 2, the string length in bytes is encoded in the first 4 bytes as a uint32
    return cppbox::BytesDecoder::peak<std::string>(peak<uint32_t>(extra_offset),
            extra_offset + ROS2MessagesTypes::fundamental::padding(sizeof(T), offset() + extra_offset) +
                    sizeof(uint32_t));
}

template<cppbox::IsDuration T>
inline T ROS2BytesDecoder::peak(const std::size_t extra_offset) const {
    // In ROS 2, duration is sec as int32_t and nanosec as uint32_t
    const std::chrono::seconds sec{peak<int32_t>(extra_offset)};
    const std::chrono::nanoseconds nanosec{peak<int32_t>(extra_offset + sizeof(int32_t))};
    return T{sec + nanosec};
}

template<cppbox::IsTimePoint T>
inline T ROS2BytesDecoder::peak(const std::size_t extra_offset) const {
    // In ROS 2, duration is sec as int32_t and nanosec as uint32_t
    const std::chrono::seconds sec{peak<int32_t>(extra_offset)};
    const std::chrono::nanoseconds nanosec{peak<uint32_t>(extra_offset + sizeof(int32_t))};
    return T{sec + nanosec};
}

template<typename T>
inline T ROS2BytesDecoder::read() {
    T out;
    read_to(out);
    return out;
}

template<typename T>
inline std::optional<T> ROS2BytesDecoder::read_optional() {
    std::optional<T> out;
    read_optional_to<T>(out);
    return out;
}

template<typename T>
inline void ROS2BytesDecoder::read_optional_to(std::optional<T>& out) {
    out = read_to<T>();
}

template<typename T>
    requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
inline void ROS2BytesDecoder::read_to(T& out) {
    // In ROS 2, fundamental types may have padding which must be ignored
    ignore_bytes(ROS2MessagesTypes::fundamental::padding(sizeof(T), offset()));
    out = cppbox::BytesDecoder::read<T>();
}

inline void ROS2BytesDecoder::read_to(std::string& out) {
    // In ROS 2, the string length in bytes is encoded in the first 4 bytes as a uint32
    out = cppbox::BytesDecoder::read<std::string>(read<uint32_t>());
    // In ROS 2, strings end with a null terminator '\0'
    [[maybe_unused]] const char null_terminator = cppbox::BytesDecoder::read<char>();
    assert(null_terminator == '\0');
}

template<cppbox::IsDuration T>
inline void ROS2BytesDecoder::read_to(T& out) {
    // In ROS 2, duration is sec as int32_t and nanosec as uint32_t
    const std::chrono::seconds sec{read<int32_t>()};
    const std::chrono::nanoseconds nanosec{read<uint32_t>()};
    out = T{sec + nanosec};
}

template<cppbox::IsTimePoint T>
inline void ROS2BytesDecoder::read_to(T& out) {
    // In ROS 2, time is sec as int32_t and nanosec as uint32_t
    const std::chrono::seconds sec{read<int32_t>()};
    const std::chrono::nanoseconds nanosec{read<uint32_t>()};
    out = T{sec + nanosec};
}

inline ROS2BytesDecoder::ROS2BytesDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_, ROS2BytesDecoder* parent_decoder_)
    : MessageDecoder<ROS2MessagesTypes, ROS2Conversions>(bytes_, size_, msg_type_, parent_decoder_) {}

inline ROS2BytesDecoder ROS2BytesDecoder::create_internal_decoder(const std::string& internal_msg_type) {
    return ROS2BytesDecoder(bytes() + offset(), internal_msg_size(internal_msg_type, 0), internal_msg_type, this);
}

template<typename T>
inline T ROS2BytesDecoder::decode_internal(const std::string& internal_msg_type) {
    T out;
    decode_internal_to(internal_msg_type, out);
    return out;
}

template<typename T>
inline void ROS2BytesDecoder::decode_internal_to(const std::string& internal_msg_type, T& out) {
    create_internal_decoder(internal_msg_type).decode_to(out);
}

template<typename T>
inline std::vector<T> ROS2BytesDecoder::decode_vector(const std::string& internal_msg_type) {
    std::vector<T> out;
    decode_vector_to(internal_msg_type, out);
    return out;
}

template<typename T>
inline void ROS2BytesDecoder::decode_vector_to(const std::string& vector_msg_type, std::vector<T>& out) {
    // ROS 2 dynamic-sized vectors store the number elements as a uint32_t in the first 4 bytes
    out.resize(read<uint32_t>());
    for (T& out_element : out) {
        decode_internal_to(vector_msg_type, out_element);
    }
}

}

#if SENSORBOX_HEADER_ONLY
#include "sensorbox/impl/ros2_decoder.impl.hpp"
#endif

#endif
