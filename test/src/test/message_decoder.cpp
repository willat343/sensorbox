#include "sensorbox/message_decoder.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "sensorbox/ros1_decoder.hpp"
#include "sensorbox/ros2_decoder.hpp"

namespace sensorbox {

TEST(message_decoder, message_is_array_type) {
    static_assert(message_is_array_type("float64[36]"));
    static_assert(message_is_array_type("geometry_msgs/Point[4]"));
    static_assert(!message_is_array_type("float64[]"));
    static_assert(!message_is_array_type("float64"));
    static_assert(!message_is_array_type("float64[a]"));
    static_assert(!message_is_array_type("float64[4"));
    static_assert(!message_is_array_type("float64 4]"));
    EXPECT_TRUE(message_is_array_type("float64[36]"));
    EXPECT_FALSE(message_is_array_type("float64[]"));
}

TEST(message_decoder, message_array_size) {
    // Only a msg_type that is not an array type can be evaluated at compile time, because std::from_chars is not
    // constexpr until C++23
    static_assert(message_array_size("float64[]") == 0);
    static_assert(message_array_size("float64") == 0);
    EXPECT_EQ(message_array_size("float64[36]"), 36);
    EXPECT_EQ(message_array_size("bool[4]"), 4);
    EXPECT_EQ(message_array_size("float64[9]"), 9);
    EXPECT_EQ(message_array_size("float64[]"), 0);
    EXPECT_EQ(message_array_size("float64"), 0);
}

TEST(message_decoder, message_array_type) {
    static_assert(message_array_type("float64[36]") == std::string_view{"float64"});
    static_assert(message_array_type("geometry_msgs/Point[4]") == std::string_view{"geometry_msgs/Point"});
    static_assert(message_array_type("float64") == std::string_view{"float64"});
    // A vector type is not an array type, so it is returned unchanged
    static_assert(message_array_type("float64[]") == std::string_view{"float64[]"});
    EXPECT_EQ(message_array_type("float64[36]"), std::string_view{"float64"});
}

TEST(message_decoder, message_is_vector_type) {
    static_assert(message_is_vector_type("float64[]"));
    static_assert(message_is_vector_type("geometry_msgs/Point[]"));
    static_assert(!message_is_vector_type("float64[4]"));
    static_assert(!message_is_vector_type("float64"));
    EXPECT_TRUE(message_is_vector_type("uint8[]"));
    EXPECT_FALSE(message_is_vector_type("uint8[3]"));
}

TEST(message_decoder, message_vector_type) {
    static_assert(message_vector_type("float64[]") == std::string_view{"float64"});
    static_assert(message_vector_type("geometry_msgs/Point[]") == std::string_view{"geometry_msgs/Point"});
    static_assert(message_vector_type("float64") == std::string_view{"float64"});
    static_assert(message_vector_type("float64[4]") == std::string_view{"float64[4]"});
    EXPECT_EQ(message_vector_type("uint8[]"), std::string_view{"uint8"});
}

TEST(message_decoder, message_contains_dynamic_field) {
    // Fundamental types, and messages built only from them, have no data-dependent length
    static_assert(!message_contains_dynamic_field<ROS1MessagesTypes>("float64"));
    static_assert(!message_contains_dynamic_field<ROS1MessagesTypes>("time"));
    EXPECT_FALSE(message_contains_dynamic_field<ROS1MessagesTypes>("geometry_msgs/Quaternion"));
    EXPECT_FALSE(message_contains_dynamic_field<ROS1MessagesTypes>("geometry_msgs/Vector3"));
    EXPECT_FALSE(message_contains_dynamic_field<ROS1MessagesTypes>("geometry_msgs/Wrench"));
    // A fixed-size array of a fundamental type is not dynamic, but a vector of one is
    EXPECT_FALSE(message_contains_dynamic_field<ROS1MessagesTypes>("float64[36]"));
    EXPECT_FALSE(message_contains_dynamic_field<ROS1MessagesTypes>("geometry_msgs/TwistWithCovariance"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("float64[]"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("string"));
    // A std_msgs/Header contains a string, so every message containing one is dynamic
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("std_msgs/Header"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("geometry_msgs/PoseStamped"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("sensor_msgs/Imu"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("anymal_msgs/Contact[]"));
    // An unknown message type is assumed to contain a dynamic field
    EXPECT_TRUE(message_contains_dynamic_field<ROS1MessagesTypes>("not_a_pkg/NotAMessage"));
    // The property is of the message definition only, so it holds for the ROS 2 definitions too
    EXPECT_FALSE(message_contains_dynamic_field<ROS2MessagesTypes>("geometry_msgs/Quaternion"));
    EXPECT_TRUE(message_contains_dynamic_field<ROS2MessagesTypes>("std_msgs/Header"));
}

TEST(message_decoder, message_fields) {
    const auto header_fields = message_fields(ROS1MessagesTypes::msg_types, "std_msgs/Header");
    ASSERT_EQ(header_fields.size(), 3);
    EXPECT_EQ(header_fields[0].type, std::string_view{"uint32"});
    EXPECT_EQ(header_fields[0].name, std::string_view{"seq"});
    EXPECT_EQ(header_fields[1].type, std::string_view{"time"});
    EXPECT_EQ(header_fields[2].type, std::string_view{"string"});
    EXPECT_TRUE(message_fields(ROS1MessagesTypes::msg_types, "not_a_pkg/NotAMessage").empty());
    EXPECT_TRUE(message_fields(ROS1MessagesTypes::msg_types, "std_msgs/Empty").empty());
}

TEST(message_decoder, message_starts_with) {
    EXPECT_EQ(message_starts_with(ROS1MessagesTypes::msg_types, "sensor_msgs/Imu"),
            std::string_view{"std_msgs/Header"});
    EXPECT_EQ(message_starts_with(ROS1MessagesTypes::msg_types, "geometry_msgs/Point"), std::string_view{"float64"});
    EXPECT_EQ(message_starts_with(ROS1MessagesTypes::msg_types, "not_a_pkg/NotAMessage"), std::string_view{});
    // A message type without any fields does not start with any message type
    EXPECT_EQ(message_starts_with(ROS1MessagesTypes::msg_types, "std_msgs/Empty"), std::string_view{});
}

TEST(ros1_messages_types, fundamental_sizes) {
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("bool"), 1);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("int8"), 1);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("uint32"), 4);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("float32"), 4);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("float64"), 8);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("time"), 8);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("duration"), 8);
    // Non-fundamental types have no known size
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("string"), 0);
    EXPECT_EQ(ROS1MessagesTypes::fundamental::size("std_msgs/Header"), 0);
}

TEST(ros1_messages_types, starts_with) {
    EXPECT_EQ(ROS1MessagesTypes::starts_with("nav_msgs/Odometry"), std::string_view{"std_msgs/Header"});
    EXPECT_EQ(ROS1MessagesTypes::starts_with("not_a_pkg/NotAMessage"), std::string_view{});
}

TEST(ros1_messages_types, all_referenced_field_types_are_known) {
    for (const MessageType& msg_type : ROS1MessagesTypes::msg_types) {
        for (const MessageField& field : msg_type.fields) {
            const std::string_view element_type = message_is_vector_type(field.type) ? message_vector_type(field.type)
                                                                                     : message_array_type(field.type);
            const bool is_known = ROS1MessagesTypes::fundamental::size(element_type) > 0 ||
                                  element_type == std::string_view{"string"} ||
                                  !message_fields(ROS1MessagesTypes::msg_types, element_type).empty();
            EXPECT_TRUE(is_known) << msg_type.type << "." << field.name << " has unknown type " << element_type;
        }
    }
}

TEST(ros1_decoder, is_decodable) {
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable<ImuMeasurement<3>>()));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable<PoseMeasurement<3>>()));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable<ActuatorMeasurements>()));
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable<ContactClassifications>());
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable<TemporalMeasurement>());
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable<std::string>());
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable<Eigen::Vector3d>());
    // There is no msg type that can be decoded to these types
    EXPECT_FALSE((ROS1BytesDecoder::is_decodable<ImuMeasurement<2>>()));
    EXPECT_FALSE((ROS1BytesDecoder::is_decodable<std::vector<double>>()));
}

TEST(ros1_decoder, is_directly_decodable_to) {
    EXPECT_TRUE((ROS1BytesDecoder::is_directly_decodable_to<ImuMeasurement<3>>("sensor_msgs/Imu")));
    EXPECT_TRUE((ROS1BytesDecoder::is_directly_decodable_to<PoseMeasurement<3>>("geometry_msgs/PoseStamped")));
    EXPECT_TRUE(ROS1BytesDecoder::is_directly_decodable_to<TemporalSpatialMeasurement>("std_msgs/Header"));
    EXPECT_FALSE((ROS1BytesDecoder::is_directly_decodable_to<ImuMeasurement<3>>("any_msgs/ImuWithTrigger")));
    EXPECT_FALSE((ROS1BytesDecoder::is_directly_decodable_to<ImuMeasurement<3>>("nav_msgs/Odometry")));
}

TEST(ros1_decoder, is_start_decodable_to) {
    // An any_msgs/ImuWithTrigger starts with a sensor_msgs/Imu
    EXPECT_TRUE((ROS1BytesDecoder::is_start_decodable_to<ImuMeasurement<3>>("any_msgs/ImuWithTrigger")));
    // A sensor_msgs/Imu starts with a std_msgs/Header
    EXPECT_TRUE(ROS1BytesDecoder::is_start_decodable_to<TemporalMeasurement>("sensor_msgs/Imu"));
    EXPECT_FALSE((ROS1BytesDecoder::is_start_decodable_to<ImuMeasurement<3>>("sensor_msgs/Imu")));
}

TEST(ros1_decoder, is_start_recursively_decodable_to) {
    // A series_elastic_actuator_msgs/SeActuatorReading starts with a std_msgs/Header
    EXPECT_TRUE(ROS1BytesDecoder::is_start_recursively_decodable_to<TemporalMeasurement>(
            "series_elastic_actuator_msgs/SeActuatorReading"));
    // A geometry_msgs/PoseStamped starts with a std_msgs/Header, which starts with a uint32
    EXPECT_FALSE(ROS1BytesDecoder::is_start_recursively_decodable_to<TemporalMeasurement>("std_msgs/Header"));
    EXPECT_FALSE(ROS1BytesDecoder::is_start_recursively_decodable_to<TemporalMeasurement>("not_a_pkg/NotAMessage"));
}

TEST(ros1_decoder, is_decodable_to) {
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<ImuMeasurement<3>>("sensor_msgs/Imu")));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<ImuMeasurement<3>>("any_msgs/ImuWithTrigger")));
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable_to<TemporalMeasurement>("std_msgs/Header"));
    EXPECT_TRUE(ROS1BytesDecoder::is_decodable_to<TemporalMeasurement>("sensor_msgs/Imu"));
    EXPECT_FALSE((ROS1BytesDecoder::is_decodable_to<ImuMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_FALSE((ROS1BytesDecoder::is_decodable_to<ImuMeasurement<3>>("not_a_pkg/NotAMessage")));
}

TEST(ros1_decoder, is_vector_or_start_recursively_decodable_to) {
    // A tf2_msgs/TFMessage is a vector of geometry_msgs/TransformStamped
    EXPECT_TRUE(
            (ROS1BytesDecoder::is_vector_or_start_recursively_decodable_to<PoseMeasurement<3>>("tf2_msgs/TFMessage")));
    EXPECT_TRUE((ROS1BytesDecoder::is_vector_or_start_recursively_decodable_to<PoseMeasurement<3>>(
            "geometry_msgs/TransformStamped[]")));
    EXPECT_FALSE((ROS1BytesDecoder::is_vector_or_start_recursively_decodable_to<PoseMeasurement<3>>(
            "geometry_msgs/PoseStamped")));
}

TEST(ros2_decoder, is_decodable_to) {
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable<ImuMeasurement<3>>()));
    EXPECT_TRUE((ROS2BytesDecoder::is_directly_decodable_to<ImuMeasurement<3>>("sensor_msgs/Imu")));
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PoseMeasurement<3>>("geometry_msgs/PoseStamped")));
    EXPECT_TRUE(ROS2BytesDecoder::is_decodable_to<TemporalSpatialMeasurement>("std_msgs/Header"));
    EXPECT_FALSE((ROS2BytesDecoder::is_decodable_to<ImuMeasurement<3>>("nav_msgs/Odometry")));
    // The ROS 2 decoder does not support the actuator and contact msg types
    EXPECT_FALSE(ROS2BytesDecoder::is_decodable<ActuatorMeasurements>());
    EXPECT_FALSE(ROS2BytesDecoder::is_decodable<ContactClassifications>());
}

TEST(ros2_decoder, string_length_includes_null_terminator) {
    // A std_msgs/String message consists of the CDR header followed by the uint32 length of the string including its
    // null terminator, the characters of the string, and the null terminator
    const std::string string{"imu_link"};
    std::vector<std::byte> bytes{ROS2MessagesTypes::cdr_header.begin(), ROS2MessagesTypes::cdr_header.end()};
    const uint32_t length = static_cast<uint32_t>(string.size() + 1);
    for (std::size_t i = 0; i < sizeof(length); ++i) {
        bytes.push_back(static_cast<std::byte>((length >> (8 * i)) & 0xFF));
    }
    for (const char c : string) {
        bytes.push_back(static_cast<std::byte>(c));
    }
    bytes.push_back(std::byte{0});
    ROS2BytesDecoder decoder{bytes.data(), bytes.size(), "std_msgs/String"};
    const std::string decoded = decoder.decode<std::string>();
    EXPECT_EQ(decoded, string);
    EXPECT_EQ(decoded.size(), string.size());
}

TEST(ros2_decoder, string_length_without_null_terminator_throws) {
    std::vector<std::byte> bytes{ROS2MessagesTypes::cdr_header.begin(), ROS2MessagesTypes::cdr_header.end()};
    bytes.insert(bytes.end(), sizeof(uint32_t), std::byte{0});  // length of zero
    ROS2BytesDecoder decoder{bytes.data(), bytes.size(), "std_msgs/String"};
    EXPECT_THROW(decoder.decode<std::string>(), std::runtime_error);
}

TEST(ros2_messages_types, fundamental_sizes_and_padding) {
    EXPECT_EQ(ROS2MessagesTypes::fundamental::size("bool"), 1);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::size("uint32"), 4);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::size("float64"), 8);
    // In ROS 2, fundamental types are aligned to multiples of their own size
    EXPECT_EQ(ROS2MessagesTypes::fundamental::padding(8, 0), 0);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::padding(8, 4), 4);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::padding(4, 2), 2);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::padding(1, 3), 0);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::padding(0, 3), 0);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::aligned_offset(8, 4), 8);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::aligned_offset(4, 4), 4);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::aligned_offset(0, 3), 3);
    // The size includes the leading padding required at the given offset
    EXPECT_EQ(ROS2MessagesTypes::fundamental::size("float64", 4), 12);
    EXPECT_EQ(ROS2MessagesTypes::fundamental::size("float64", 8), 8);
}

TEST(ros2_messages_types, builtin_interfaces_size) {
    EXPECT_EQ(ROS2MessagesTypes::builtin_interfaces::size("builtin_interfaces/Time"), 8);
    EXPECT_EQ(ROS2MessagesTypes::builtin_interfaces::size("builtin_interfaces/Duration"), 8);
    EXPECT_EQ(ROS2MessagesTypes::builtin_interfaces::size("not_a_pkg/NotAMessage"), 0);
}

TEST(ros2_messages_types, remove_internal_msg_substring) {
    EXPECT_EQ(ROS2MessagesTypes::remove_internal_msg_substring("sensor_msgs/msg/Imu"), "sensor_msgs/Imu");
    EXPECT_EQ(ROS2MessagesTypes::remove_internal_msg_substring("sensor_msgs/Imu"), "sensor_msgs/Imu");
}

TEST(ros2_messages_types, all_referenced_field_types_are_known) {
    for (const MessageType& msg_type : ROS2MessagesTypes::msg_types) {
        for (const MessageField& field : msg_type.fields) {
            const std::string_view element_type = message_is_vector_type(field.type) ? message_vector_type(field.type)
                                                                                     : message_array_type(field.type);
            const bool is_known = ROS2MessagesTypes::fundamental::size(element_type) > 0 ||
                                  element_type == std::string_view{"string"} ||
                                  ROS2MessagesTypes::builtin_interfaces::size(element_type) > 0 ||
                                  !message_fields(ROS2MessagesTypes::msg_types, element_type).empty();
            EXPECT_TRUE(is_known) << msg_type.type << "." << field.name << " has unknown type " << element_type;
        }
    }
}

}
