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
            throw_here("Decoding failed because T cannot be decoded to from message type " + msg_type() + ".");
        }
        assert(is_finished());
    } else {
        throw_here("Decoding failed because T is not decodable from any message type.");
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

inline constexpr bool ROS1BytesDecoder::is_decodable(const std::string_view msg_type) {
    constexpr auto decodable_msg_types = std::to_array<std::string_view>({"duration", "string", "time",
            "std_msgs/Duration", "std_msgs/Header", "std_msgs/String", "std_msgs/Time", "geometry_msgs/Point",
            "geometry_msgs/Pose", "geometry_msgs/PoseStamped", "geometry_msgs/PoseWithCovariance",
            "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/Quaternion", "geometry_msgs/Transform",
            "geometry_msgs/TransformStamped", "geometry_msgs/Twist", "geometry_msgs/TwistStamped",
            "geometry_msgs/TwistWithCovariance", "geometry_msgs/TwistWithCovarianceStamped", "geometry_msgs/Vector3",
            "nav_msgs/Odometry", "sensor_msgs/Imu", "tf2_msgs/TFMessage", "anymal_msgs/AnymalState",
            "series_elastic_actuator_msgs/SeActuatorReadings", "series_elastic_actuator_msgs/SeActuatorReading"});
    return std::find(decodable_msg_types.cbegin(), decodable_msg_types.cend(), msg_type) != decodable_msg_types.cend();
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

inline constexpr std::string_view ROS1BytesDecoder::starts_with(const std::string_view msg_type) {
    // All message types from the following packages are included in the map:
    //  - std_msgs
    //  - geometry_msgs
    //  - nav_msgs
    //  - sensor_msgs
    //  - tf2_msgs
    //  - any_msgs (ANYbotics)
    //  - anymal_msgs (ANYbotics)
    //  - series_elastic_actuator_msgs (ANYbotics)
    constexpr auto map = std::to_array<std::pair<std::string_view, std::string_view>>({{"std_msgs/Bool", "bool"},
            {"std_msgs/Byte", "byte"}, {"std_msgs/ByteMultiArray", "std_msgs/MultiArrayLayout"},
            {"std_msgs/Char", "char"}, {"std_msgs/ColorRGBA", "float32"}, {"std_msgs/Duration", "duration"},
            {"std_msgs/Empty", ""}, {"std_msgs/Float32", "float32"},
            {"std_msgs/Float32MultiArray", "std_msgs/MultiArrayLayout"}, {"std_msgs/Float64", "float64"},
            {"std_msgs/Float64MultiArray", "std_msgs/MultiArrayLayout"}, {"std_msgs/Header", "uint32"},
            {"std_msgs/Int16", "int16"}, {"std_msgs/Int16MultiArray", "std_msgs/MultiArrayLayout"},
            {"std_msgs/Int32", "int32"}, {"std_msgs/Int32MultiArray", "std_msgs/MultiArrayLayout"},
            {"std_msgs/Int64", "int64"}, {"std_msgs/Int64MultiArray", "std_msgs/MultiArrayLayout"},
            {"std_msgs/Int8", "int8"}, {"std_msgs/Int8MultiArray", "std_msgs/MultiArrayLayout"},
            {"std_msgs/MultiArrayDimension", "string"}, {"std_msgs/MultiArrayLayout", "std_msgs/MultiArrayDimension[]"},
            {"std_msgs/String", "string"}, {"std_msgs/Time", "time"}, {"std_msgs/UInt16", "uint16"},
            {"std_msgs/UInt16MultiArray", "std_msgs/MultiArrayLayout"}, {"std_msgs/UInt32", "uint32"},
            {"std_msgs/UInt32MultiArray", "std_msgs/MultiArrayLayout"}, {"std_msgs/UInt64", "uint64"},
            {"std_msgs/UInt64MultiArray", "std_msgs/MultiArrayLayout"}, {"std_msgs/UInt8", "uint8"},
            {"geometry_msgs/Accel", "geometry_msgs/Vector3"}, {"geometry_msgs/AccelStamped", "std_msgs/Header"},
            {"geometry_msgs/AccelWithCovariance", "geometry_msgs/Accel"},
            {"geometry_msgs/AccelWithCovarianceStamped", "std_msgs/Header"}, {"geometry_msgs/Inertia", "float64"},
            {"geometry_msgs/InertiaStamped", "std_msgs/Header"}, {"geometry_msgs/Point", "float64"},
            {"geometry_msgs/Point32", "float32"}, {"geometry_msgs/PointStamped", "std_msgs/Header"},
            {"geometry_msgs/Polygon", "geometry_msgs/Point32[]"}, {"geometry_msgs/PolygonStamped", "std_msgs/Header"},
            {"geometry_msgs/Pose", "geometry_msgs/Point"}, {"geometry_msgs/Pose2D", "float64"},
            {"geometry_msgs/PoseArray", "std_msgs/Header"}, {"geometry_msgs/PoseStamped", "std_msgs/Header"},
            {"geometry_msgs/PoseWithCovariance", "geometry_msgs/Pose"},
            {"geometry_msgs/PoseWithCovarianceStamped", "std_msgs/Header"}, {"geometry_msgs/Quaternion", "float64"},
            {"geometry_msgs/QuaternionStamped", "std_msgs/Header"},
            {"geometry_msgs/Transform", "geometry_msgs/Vector3"}, {"geometry_msgs/TransformStamped", "std_msgs/Header"},
            {"geometry_msgs/Twist", "geometry_msgs/Vector3"}, {"geometry_msgs/TwistStamped", "std_msgs/Header"},
            {"geometry_msgs/TwistWithCovariance", "geometry_msgs/Twist"},
            {"geometry_msgs/TwistWithCovarianceStamped", "std_msgs/Header"}, {"geometry_msgs/Vector3", "float64"},
            {"geometry_msgs/Vector3Stamped", "std_msgs/Header"}, {"geometry_msgs/Wrench", "geometry_msgs/Vector3"},
            {"geometry_msgs/WrenchStamped", "std_msgs/Header"}, {"nav_msgs/GridCells", "std_msgs/Header"},
            {"nav_msgs/MapMetaData", "time"}, {"nav_msgs/OccupancyGrid", "std_msgs/Header"},
            {"nav_msgs/Odometry", "std_msgs/Header"}, {"nav_msgs/Path", "std_msgs/Header"},
            {"sensor_msgs_msgs/BatteryState", "std_msgs/Header"}, {"sensor_msgs_msgs/CameraInfo", "std_msgs/Header"},
            {"sensor_msgs_msgs/ChannelFloat32", "string"}, {"sensor_msgs_msgs/CompressedImage", "std_msgs/Header"},
            {"sensor_msgs_msgs/FluidPressure", "std_msgs/Header"}, {"sensor_msgs_msgs/Illuminance", "std_msgs/Header"},
            {"sensor_msgs_msgs/Image", "std_msgs/Header"}, {"sensor_msgs_msgs/Imu", "std_msgs/Header"},
            {"sensor_msgs_msgs/JointState", "std_msgs/Header"}, {"sensor_msgs_msgs/Joy", "std_msgs/Header"},
            {"sensor_msgs_msgs/JoyFeedback", "uint8"},
            {"sensor_msgs_msgs/JoyFeedbackArray", "sensor_msgs_msgs/JoyFeedback[]"},
            {"sensor_msgs_msgs/LaserEcho", "float32[]"}, {"sensor_msgs_msgs/LaserScan", "std_msgs/Header"},
            {"sensor_msgs_msgs/MagneticField", "std_msgs/Header"},
            {"sensor_msgs_msgs/MultiDOFJointState", "std_msgs/Header"},
            {"sensor_msgs_msgs/MultiEchoLaserScan", "std_msgs/Header"},
            {"sensor_msgs_msgs/NavSatFix", "std_msgs/Header"}, {"sensor_msgs_msgs/NavSatStatus", "int8"},
            {"sensor_msgs_msgs/PointCloud", "std_msgs/Header"}, {"sensor_msgs_msgs/PointCloud2", "std_msgs/Header"},
            {"sensor_msgs_msgs/PointField", "string"}, {"sensor_msgs_msgs/Range", "std_msgs/Header"},
            {"sensor_msgs_msgs/RegionOfInterest", "uint32"}, {"sensor_msgs_msgs/RelativeHumidity", "std_msgs/Header"},
            {"sensor_msgs_msgs/Temperature", "std_msgs/Header"}, {"sensor_msgs_msgs/TimeReference", "std_msgs/Header"},
            {"tf2_msgs/TF2Error", "uint8"}, {"tf2_msgs/TFMessage", "geometry_msgs/TransformStamped[]"},
            {"any_msgs/BoolStamped", "std_msgs/Header"}, {"any_msgs/Event", "time"},
            {"any_msgs/ExtendedJointState", "std_msgs/Header"}, {"any_msgs/Float64Stamped", "std_msgs/Header"},
            {"any_msgs/ImuWithTrigger", "sensor_msgs/Imu"}, {"any_msgs/PointContact", "std_msgs/Header"},
            {"any_msgs/SensorTimeInfo", "std_msgs/Header"}, {"any_msgs/State", "time"},
            {"any_msgs/UserInteractionOption", "string"}, {"anymal_msgs/AnymalState", "std_msgs/Header"},
            {"anymal_msgs/BMSState", "uint16[]"}, {"anymal_msgs/Contact", "std_msgs/Header"},
            {"anymal_msgs/Contacts", "anymal_msgs/Contact[]"}, {"anymal_msgs/GaitPattern", "float64"},
            {"anymal_msgs/GaitPatterns", "std_msgs/Header"}, {"anymal_msgs/LegAttributes", "bool[4]"},
            {"series_elastic_actuator_msgs/SeActuatorCommand", "std_msgs/Header"},
            {"series_elastic_actuator_msgs/SeActuatorCommands", "series_elastic_actuator_msgs/SeActuatorCommand[]"},
            {"series_elastic_actuator_msgs/SeActuatorReading", "std_msgs/Header"},
            {"series_elastic_actuator_msgs/SeActuatorReadingExtended", "std_msgs/Header"},
            {"series_elastic_actuator_msgs/SeActuatorReadings", "series_elastic_actuator_msgs/SeActuatorReading[]"},
            {"series_elastic_actuator_msgs/SeActuatorReadingsExtended",
                    "series_elastic_actuator_msgs/SeActuatorReadingExtended[]"},
            {"series_elastic_actuator_msgs/SeActuatorState", "std_msgs/Header"},
            {"series_elastic_actuator_msgs/SeActuatorStateExtended", "std_msgs/Header"},
            {"series_elastic_actuator_msgs/SeActuatorStates", "series_elastic_actuator_msgs/SeActuatorState[]"}});
    const auto it =
            std::find_if(map.cbegin(), map.cend(), [msg_type](const auto& pair) { return pair.first == msg_type; });
    return it != map.cend() ? it->second : std::string_view();
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
