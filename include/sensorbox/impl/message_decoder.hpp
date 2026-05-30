#ifndef SENSORBOX_IMPL_MESSAGE_DECODER_HPP
#define SENSORBOX_IMPL_MESSAGE_DECODER_HPP

#include <cppbox/array.hpp>

#include "sensorbox/message_decoder.hpp"

namespace sensorbox {

template<class MessageTypes, class Conversions>
inline MessageDecoder<MessageTypes, Conversions>::MessageDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_)
    : MessageDecoder(bytes_, size_, msg_type_, nullptr) {}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_decodable() {
    return Conversions::template decodable_msg_types<T>().empty();
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_decodable_to(const std::string_view msg_type) {
    return is_directly_decodable_to<T>(msg_type) || is_vector_or_start_recursively_decodable_to<T>(msg_type);
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_decodable_to() const {
    return is_decodable_to<T>(msg_type());
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_directly_decodable_to(
        const std::string_view msg_type) {
    return cppbox::contains(Conversions::template decodable_msg_types<T>(), msg_type);
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_directly_decodable_to() const {
    return is_directly_decodable_to<T>(msg_type());
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_start_decodable_to(
        const std::string_view msg_type) {
    return is_directly_decodable_to<T>(MessageTypes::starts_with(msg_type));
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_start_decodable_to() const {
    return is_start_decodable_to<T>(msg_type());
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_start_recursively_decodable_to(
        const std::string_view msg_type) {
    return msg_type != std::string_view() &&
           (is_directly_decodable_to<T>(MessageTypes::starts_with(msg_type)) ||
                   is_start_recursively_decodable_to<T>(MessageTypes::starts_with(msg_type)));
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_start_recursively_decodable_to() const {
    return is_start_recursively_decodable_to<T>(msg_type());
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_vector_or_start_recursively_decodable_to(
        const std::string_view msg_type) {
    return msg_type != std::string_view() &&
           ((message_is_vector_type(msg_type) &&
                    (is_directly_decodable_to<T>(message_vector_type(msg_type)) ||
                            is_vector_or_start_recursively_decodable_to<T>(message_vector_type(msg_type)))) ||
                   (!message_is_vector_type(msg_type) &&
                           (is_directly_decodable_to<T>(MessageTypes::starts_with(msg_type)) ||
                                   is_vector_or_start_recursively_decodable_to<T>(
                                           MessageTypes::starts_with(msg_type)))));
}

template<class MessageTypes, class Conversions>
template<typename T>
inline constexpr bool MessageDecoder<MessageTypes, Conversions>::is_vector_or_start_recursively_decodable_to() const {
    return is_vector_or_start_recursively_decodable_to<T>(msg_type());
}

template<class MessageTypes, class Conversions>
inline const std::string& MessageDecoder<MessageTypes, Conversions>::msg_type() const {
    return msg_type_;
}

template<class MessageTypes, class Conversions>
inline std::string_view MessageDecoder<MessageTypes, Conversions>::starts_with() const {
    return message_starts_with(MessageTypes::msg_types, msg_type());
}

template<class MessageTypes, class Conversions>
inline bool MessageDecoder<MessageTypes, Conversions>::is_vector_type() const {
    return message_is_vector(MessageTypes::msg_types, msg_type());
}

template<class MessageTypes, class Conversions>
inline MessageDecoder<MessageTypes, Conversions>::MessageDecoder(const std::byte* bytes_, const std::size_t size_,
        const std::string& msg_type_, MessageDecoder* parent_decoder_)
    : cppbox::BytesDecoder(bytes_, size_, parent_decoder_), msg_type_(msg_type_) {}

}

#endif
