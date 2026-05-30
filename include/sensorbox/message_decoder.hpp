#ifndef SENSORBOX_MESSAGE_DECODER_HPP
#define SENSORBOX_MESSAGE_DECODER_HPP

#include <charconv>
#include <span>
#include <string_view>
#include <utility>

namespace sensorbox {

struct MessageField {
    std::string_view type;
    std::string_view name;
};

struct MessageType {
    std::string_view type;
    std::span<const MessageField> fields;
};

struct MessageSize {
    std::string_view type;
    std::size_t size;
};

template<std::size_t Size>
constexpr std::span<const MessageField> message_fields(const std::array<MessageType, Size>& msg_types,
        const std::string_view msg_type) {
    const auto it = std::find_if(msg_types.cbegin(), msg_types.cend(),
            [msg_type](const MessageType& message_size) { return message_size.type == msg_type; });
    return it != msg_types.cend() ? it->fields : std::span<const MessageField>();
}

template<std::size_t Size>
constexpr std::string_view message_starts_with(const std::array<MessageType, Size>& msg_types,
        const std::string_view msg_type) {
    const auto it = std::find_if(msg_types.cbegin(), msg_types.cend(),
            [msg_type](const MessageType& message_type) { return message_type.type == msg_type; });
    return it != msg_types.cend() ? it->fields.front().type : std::string_view();
}

constexpr bool message_is_array_type(const std::string_view msg_type) {
    const std::size_t open_bracket = msg_type.find_last_of('[');
    if (open_bracket == std::string_view::npos || !msg_type.ends_with("]")) {
        return false;
    }
    // Must be at least one number in between the brackets
    const std::string_view array_size_string = msg_type.substr(open_bracket + 1, msg_type.size() - open_bracket - 2);
    for (const char c : array_size_string) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    return !array_size_string.empty();
}

constexpr std::size_t message_array_size(const std::string_view msg_type) {
    if (!message_is_array_type(msg_type)) {
        return 0;
    }
    const std::size_t open_bracket = msg_type.find_last_of('[');
    const std::string_view array_size_string = msg_type.substr(open_bracket + 1, msg_type.size() - open_bracket - 2);
    std::size_t array_size{0};
    const auto result =
            std::from_chars(array_size_string.data(), array_size_string.data() + array_size_string.size(), array_size);
    return result.ec == std::errc() ? array_size : 0;
}

constexpr std::string_view message_array_type(const std::string_view msg_type) {
    return message_is_array_type(msg_type) ? msg_type.substr(0, msg_type.find_last_of('[')) : msg_type;
}

constexpr bool message_is_vector_type(const std::string_view msg_type) {
    return msg_type.ends_with("[]");
}

constexpr std::string_view message_vector_type(const std::string_view msg_type) {
    return message_is_vector_type(msg_type) ? msg_type.substr(0, msg_type.size() - 2) : msg_type;
}

template<class MessageTypes, class Conversions>
class MessageDecoder : public cppbox::BytesDecoder {
public:
    /**
     * @brief Construct a new message-based bytes decoder.
     *
     * @param bytes_
     * @param size_
     * @param msg_type_
     */
    explicit MessageDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_);

    /**
     * @brief Check if there is any msg type that can be decoded to type T according to `Conversions`.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_decodable();

    /**
     * @brief Check if `msg_type` is decodable to type `T`. A message type is decodable to type `T` if:
     * - it is directly decodable, or
     * - recursively the first message type is directly decodable.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_decodable_to(const std::string_view msg_type);

    template<typename T>
    constexpr bool is_decodable_to() const;

    /**
     * @brief Check if `msg_type` is directly decodable to type `T` according to `Conversions`.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_directly_decodable_to(const std::string_view msg_type);

    template<typename T>
    constexpr bool is_directly_decodable_to() const;

    /**
     * @brief Check if the first message type of `msg_type` is decodable to type `T` according to `Conversions`, using
     * `MessageTypes`.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_start_decodable_to(const std::string_view msg_type);

    template<typename T>
    constexpr bool is_start_decodable_to() const;

    /**
     * @brief Check if the first message type of `msg_type`, or its first message type recursively, is decodable to type
     * `T` according to `Conversions`, using `MessageTypes`.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_start_recursively_decodable_to(const std::string_view msg_type);

    template<typename T>
    constexpr bool is_start_recursively_decodable_to() const;

    /**
     * @brief Check if `msg_type` is a vector of elements that are decodable to type `T`, or the first message of
     * `msg_type` is, checking recursively.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_vector_or_start_recursively_decodable_to(const std::string_view msg_type);

    template<typename T>
    constexpr bool is_vector_or_start_recursively_decodable_to() const;

    /**
     * @brief Get the message type.
     *
     * @return const std::string&
     */
    const std::string& msg_type() const;

    /**
     * @brief Get the message type that `msg_type()` starts with.
     *
     * @return std::string_view
     */
    std::string_view starts_with() const;

    /**
     * @brief Check if `msg_type()` is a vector type.
     *
     * @return true
     * @return false
     */
    bool is_vector_type() const;

protected:
    explicit MessageDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
            MessageDecoder* parent_decoder_);

private:
    std::string msg_type_;
};

}

#include "sensorbox/impl/message_decoder.hpp"

#endif
