#include "sensorbox/message_decoder.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "sensorbox/ros1_decoder.hpp"
#include "sensorbox/ros2_decoder.hpp"

namespace sensorbox {

/**
 * @brief Append a value to a message in ROS 1 serialisation, as its bytes in little endian without padding, which is
 * this machine's representation.
 *
 * @param bytes serialised message
 * @param value value of a trivially copyable type, e.g. an integer or a double
 */
void append_ros1(std::vector<std::byte>& bytes, const auto value) {
    const std::byte* const begin = reinterpret_cast<const std::byte*>(&value);
    bytes.insert(bytes.end(), begin, begin + sizeof(value));
}

/**
 * @brief Append a string to a message in ROS 1 serialisation, as its length (uint32) followed by its characters.
 *
 * @param bytes serialised message
 * @param string
 */
void append_ros1_string(std::vector<std::byte>& bytes, const std::string& string) {
    append_ros1(bytes, static_cast<uint32_t>(string.size()));
    for (const char c : string) {
        bytes.push_back(static_cast<std::byte>(c));
    }
}

/**
 * @brief Append a float64 array of variable size to a message in ROS 1 serialisation, as its size (uint32) followed by
 * its values.
 *
 * @param bytes serialised message
 * @param values
 */
void append_ros1_values(std::vector<std::byte>& bytes, const std::vector<double>& values) {
    append_ros1(bytes, static_cast<uint32_t>(values.size()));
    for (const double value : values) {
        append_ros1(bytes, value);
    }
}

/**
 * @brief Get an any_msgs/ExtendedJointState message in ROS 1 serialisation: header, then the names of two joints and
 * their positions, velocities, the given accelerations and efforts.
 *
 * @param accelerations joint accelerations, one per joint in a valid message
 * @return std::vector<std::byte> serialised message
 */
std::vector<std::byte> test_extended_joint_state_bytes(const std::vector<double>& accelerations) {
    std::vector<std::byte> bytes;
    append_ros1(bytes, uint32_t{7});           // header.seq
    append_ros1(bytes, uint32_t{1700000000});  // header.stamp.sec
    append_ros1(bytes, uint32_t{250000000});   // header.stamp.nsec
    append_ros1_string(bytes, "base");         // header.frame_id
    append_ros1(bytes, uint32_t{2});           // name
    append_ros1_string(bytes, "LF_HAA");
    append_ros1_string(bytes, "LF_HFE");
    append_ros1_values(bytes, {1.0, 1.5});     // position
    append_ros1_values(bytes, {2.0, 2.5});     // velocity
    append_ros1_values(bytes, accelerations);  // acceleration
    append_ros1_values(bytes, {4.0, 4.5});     // effort
    return bytes;
}

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
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<PoseMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<PoseMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<PositionMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<PositionMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
    // Only a nav_msgs/Odometry has a twist
    EXPECT_TRUE((ROS1BytesDecoder::is_decodable_to<PoseTwistMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_FALSE(
            (ROS1BytesDecoder::is_decodable_to<PoseTwistMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
}

TEST(ros1_decoder, odometry_to_pose_twist) {
    // A nav_msgs/Odometry message in ROS 1 serialisation (little endian, without padding): header (seq, stamp,
    // frame_id), child_frame_id, pose with covariance and twist with covariance
    std::vector<std::byte> bytes;
    append_ros1(bytes, uint32_t{7});           // header.seq
    append_ros1(bytes, uint32_t{1700000000});  // header.stamp.sec
    append_ros1(bytes, uint32_t{250000000});   // header.stamp.nsec
    append_ros1_string(bytes, "map");          // header.frame_id
    append_ros1_string(bytes, "body");         // child_frame_id
    const Eigen::Vector3d position{1.0, 2.0, 3.0};
    const Eigen::Quaterniond orientation{Eigen::AngleAxisd{0.5, Eigen::Vector3d{1.0, 2.0, 3.0}.normalized()}};
    for (const double value : {position.x(), position.y(), position.z(), orientation.x(), orientation.y(),
                 orientation.z(), orientation.w()}) {
        append_ros1(bytes, value);
    }
    for (int i = 0; i < 36; ++i) {
        append_ros1(bytes, 0.01 * i);  // pose.covariance
    }
    const Eigen::Vector<double, 6> twist{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    for (int i = 0; i < 6; ++i) {
        append_ros1(bytes, twist[i]);  // twist.twist.linear, twist.twist.angular
    }
    for (int i = 0; i < 36; ++i) {
        append_ros1(bytes, 1.0 + i);  // twist.covariance
    }
    ROS1BytesDecoder decoder{bytes.data(), bytes.size(), "nav_msgs/Odometry"};
    const PoseTwistMeasurement<3> measurement = decoder.decode<PoseTwistMeasurement<3>>();
    EXPECT_EQ(measurement.timestamp(),
            PoseTwistMeasurement<3>::Timestamp{std::chrono::seconds{1700000000} + std::chrono::nanoseconds{250000000}});
    EXPECT_EQ(measurement.frame(), "map");
    EXPECT_EQ(measurement.child_frame(), "body");
    EXPECT_TRUE(measurement.pose().translation().isApprox(position));
    EXPECT_TRUE(Eigen::Quaterniond{measurement.pose().rotation()}.isApprox(orientation));
    EXPECT_TRUE(measurement.twist().isApprox(twist));
}

TEST(ros1_decoder, se_actuator_state_to_actuator_measurement) {
    // A series_elastic_actuator_msgs/SeActuatorState message in ROS 1 serialisation (little endian, without padding):
    // header, name, statusword, the actuator's quantities and an IMU message
    std::vector<std::byte> bytes;
    append_ros1(bytes, uint32_t{7});           // header.seq
    append_ros1(bytes, uint32_t{1700000000});  // header.stamp.sec
    append_ros1(bytes, uint32_t{250000000});   // header.stamp.nsec
    append_ros1_string(bytes, "base");         // header.frame_id
    append_ros1_string(bytes, "LF_HAA");       // name
    append_ros1(bytes, uint32_t{0});           // statusword
    // current, gear_position, gear_velocity, joint_position, joint_velocity, joint_acceleration and joint_torque
    for (const double value : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}) {
        append_ros1(bytes, value);
    }
    append_ros1(bytes, uint32_t{0});   // imu.header.seq
    append_ros1(bytes, uint32_t{0});   // imu.header.stamp.sec
    append_ros1(bytes, uint32_t{0});   // imu.header.stamp.nsec
    append_ros1_string(bytes, "imu");  // imu.header.frame_id
    // imu.orientation, imu.angular_velocity and imu.linear_acceleration, each followed by its float64[9] covariance
    for (int i = 0; i < 4 + 9 + 3 + 9 + 3 + 9; ++i) {
        append_ros1(bytes, 0.0);
    }
    ROS1BytesDecoder decoder{bytes.data(), bytes.size(), "series_elastic_actuator_msgs/SeActuatorState"};
    const ActuatorMeasurement measurement = decoder.decode<ActuatorMeasurement>();
    EXPECT_TRUE(decoder.is_finished());
    EXPECT_EQ(measurement.timestamp(),
            ActuatorMeasurement::Timestamp{std::chrono::seconds{1700000000} + std::chrono::nanoseconds{250000000}});
    EXPECT_EQ(measurement.frame(), "base");
    EXPECT_EQ(measurement.name(), "LF_HAA");
    EXPECT_EQ(measurement.actuator_type(), ActuatorType::SERIES_ELASTIC);
    EXPECT_EQ(measurement.current(), 1.0);
    EXPECT_EQ(measurement.motor_position(), 2.0);
    EXPECT_EQ(measurement.motor_velocity(), 3.0);
    EXPECT_EQ(measurement.joint_position(), 4.0);
    EXPECT_EQ(measurement.joint_velocity(), 5.0);
    EXPECT_EQ(measurement.joint_acceleration(), 6.0);
    EXPECT_EQ(measurement.joint_torque(), 7.0);
}

TEST(ros1_decoder, extended_joint_state_to_actuator_measurements) {
    const std::vector<std::byte> bytes = test_extended_joint_state_bytes({3.0, 3.5});
    ROS1BytesDecoder decoder{bytes.data(), bytes.size(), "any_msgs/ExtendedJointState"};
    const ActuatorMeasurements measurements = decoder.decode<ActuatorMeasurements>();
    EXPECT_TRUE(decoder.is_finished());
    EXPECT_EQ(measurements.timestamp(),
            ActuatorMeasurements::Timestamp{std::chrono::seconds{1700000000} + std::chrono::nanoseconds{250000000}});
    EXPECT_EQ(measurements.names(), (std::vector<std::string>{"LF_HAA", "LF_HFE"}));
    EXPECT_EQ(measurements.joint_positions(), (std::vector<double>{1.0, 1.5}));
    EXPECT_EQ(measurements.joint_velocities(), (std::vector<double>{2.0, 2.5}));
    EXPECT_EQ(measurements.joint_accelerations(), (std::vector<double>{3.0, 3.5}));
    EXPECT_EQ(measurements.joint_torques(), (std::vector<double>{4.0, 4.5}));
}

TEST(ros1_decoder, extended_joint_state_with_another_number_of_accelerations_throws) {
    // Accelerations of fewer or more joints than the names
    for (const std::vector<double>& accelerations : {std::vector<double>{3.0}, std::vector<double>{3.0, 3.5, 3.75}}) {
        const std::vector<std::byte> bytes = test_extended_joint_state_bytes(accelerations);
        ROS1BytesDecoder decoder{bytes.data(), bytes.size(), "any_msgs/ExtendedJointState"};
        EXPECT_THROW(decoder.decode<ActuatorMeasurements>(), std::runtime_error);
    }
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
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PoseMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PoseMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PositionMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PositionMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
    EXPECT_TRUE((ROS2BytesDecoder::is_decodable_to<PoseTwistMeasurement<3>>("nav_msgs/Odometry")));
    EXPECT_FALSE(
            (ROS2BytesDecoder::is_decodable_to<PoseTwistMeasurement<3>>("geometry_msgs/PoseWithCovarianceStamped")));
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
