#ifndef SENSORBOX_ROS1_DECODER_HPP
#define SENSORBOX_ROS1_DECODER_HPP

#include <array>
#include <chrono>
#include <cppbox/bytes.hpp>
#include <string>
#include <vector>

#include "sensorbox/actuator.hpp"
#include "sensorbox/contact.hpp"
#include "sensorbox/imu.hpp"
#include "sensorbox/measurement.hpp"
#include "sensorbox/pose.hpp"

namespace sensorbox {

class ROS1BytesDecoder : public cppbox::BytesDecoder {
public:
    explicit ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_);

    /**
     * @brief Decode all bytes to a T object.
     *
     * @tparam T
     * @return T
     */
    template<typename T>
    T decode_to();

    /**
     * @brief Decode all bytes to a T object.
     *
     * @tparam T
     * @param out
     */
    template<typename T>
    void decode_to(T& out);

    /**
     * @brief Decode all bytes to an optional T object.
     *
     * @tparam T
     * @param out
     */
    template<typename T>
    void decode_to_optional(std::optional<T>& out);

    /**
     * @brief Ignore a string message.
     *
     */
    void ignore_string();

    /**
     * @brief Ignore a message of known type.
     *
     * @param msg_type
     */
    void ignore(const std::string_view msg_type);

    /**
     * @brief Ignore a vector of messages of known type.
     *
     * @param msg_type
     */
    void ignore_vector(const std::string_view msg_type);

    /**
     * @brief Ignore a vector of trivial type messages.
     *
     * @tparam T
     */
    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    void ignore_vector();

    /**
     * @brief Check if there is support for decoding `msg_type`.
     *
     * @param msg_type
     * @return true
     * @return false
     */
    static constexpr bool is_decodable(const std::string_view msg_type);

    /**
     * @brief Check if there is any msg type that can be decoded to type T.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_decodable();

    /**
     * @brief Check if `msg_type` is decodable to type `T`. A message type is decodable to type `T` if it is directly
     * decodable as one of the message types in `ROS1DecodabilityTraits<T>::msg_types`, or if recursively the first
     * message type is directly decodable.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_decodable_to(const std::string_view msg_type);

    /**
     * @brief Check if `msg_type()` is decodable to type T.
     *
     * See static version of this function for more details.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    bool is_decodable_to() const;

    /**
     * @brief Check if `msg_type` is directly decodable to type `T`. A message type is directly decodable to type `T`
     * only if it exists in the `msg_types` of the `ROS1DecodabilityTraits<T>`.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_directly_decodable_to(const std::string_view msg_type);

    /**
     * @brief Check if `msg_type()` is directly decodable to type `T`.
     *
     * See static version of this function for more details.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    bool is_directly_decodable_to() const;

    /**
     * @brief Check if the first message type of `msg_type` is decodable to type `T`, using the `msg_types` of the
     * `ROS1DecodabilityTraits<T>`.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_start_decodable_to(const std::string_view msg_type);

    /**
     * @brief Check if the first message type of `msg_type` is decodable to type `T`.
     *
     * See static version of this function for more details.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    bool is_start_decodable_to() const;

    /**
     * @brief Get the message type.
     *
     * @return const std::string&
     */
    const std::string& msg_type() const;

    /**
     * @brief Read data at the current offset plus optional extra offset without changing internal offsets used during
     * reading.
     *
     * @tparam T
     * @param extra_offset
     */
    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    T peak(const std::size_t extra_offset = 0) const;

    /**
     * @brief Read data at the current offset plus optional extra offset without changing internal offsets used during
     * reading.
     *
     * @tparam T
     * @param extra_offset
     */
    template<typename T>
        requires(std::is_same_v<T, std::string>)
    T peak(const std::size_t extra_offset = 0) const;

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    T read();

    template<typename T>
        requires(std::is_same_v<T, std::string>)
    T read();

    template<typename T>
    T read_to();

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    void read_to(T& out);

    void read_to(std::string& out);

    void read_to(std::chrono::nanoseconds& out);

    void read_to(std::chrono::steady_clock::time_point& out);

    void read_to(std::chrono::system_clock::time_point& out);

    void read_to(Eigen::Ref<Eigen::Vector3d> out);

    void read_to(Eigen::Quaterniond& out);

    void read_to(Eigen::Isometry3d& out);

    void read_to(ActuatorMeasurement& out);

    void read_to(std::vector<ActuatorMeasurement>& out);

    void read_to(ContactClassifications& out);

    void read_to(ImuMeasurement<3>& out);

    void read_to(PoseMeasurement<3>& out);

    void read_to(std::vector<PoseMeasurement<3>>& out);

    void read_to(TemporalMeasurement& out);

    void read_to(TemporalSpatialMeasurement& out);

    template<typename T>
    void read_to_optional(std::optional<T>& out);

    /**
     * @brief Return the message type that `msg_type` starts with.
     *
     * @param msg_type
     * @return constexpr std::string_view
     */
    static constexpr std::string_view starts_with(const std::string_view msg_type);

    /**
     * @brief Return the message type that `msg_type()` starts with.
     *
     * @return std::string_view
     */
    std::string_view starts_with();

    /**
     * @brief Check if `msg_type` starts with `start_msg_type`.
     *
     * @param msg_type
     * @param start_msg_type
     * @return true
     * @return false
     */
    static constexpr bool starts_with(const std::string_view msg_type, const std::string_view start_msg_type);

protected:
    explicit ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
            ROS1BytesDecoder* parent_decoder_);

    ROS1BytesDecoder create_internal_decoder(const std::string& internal_msg_type);

    template<typename T>
    void decode_internal_to(const std::string& internal_msg_type, T& out);

    template<typename T>
    void decode_vector_to(const std::string& vector_msg_type, std::vector<T>& out);

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    void ignore(const std::size_t num_ignore = 1);

    template<typename T>
        requires(std::is_same_v<T, std::string>)
    void ignore(const std::size_t num_ignore = 1);

    std::size_t internal_msg_size(const std::string_view internal_msg_type, std::size_t offset) const;

    std::size_t internal_vector_msg_size(const std::string_view internal_msg_type, std::size_t offset) const;

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    std::size_t internal_vector_msg_size(std::size_t offset) const;

private:
    const std::string msg_type_;
};

/**
 * @brief A ROS 1 message is decodable to type T if its message type exists in `msg_types` or if its first internal
 * message type starts with one of these types.
 *
 * @tparam T
 */
template<typename T>
struct ROS1DecodabilityTraits {
    static constexpr std::array<std::string_view, 0> msg_types{};
};

template<>
struct ROS1DecodabilityTraits<std::string> {
    static constexpr std::array<std::string_view, 2> msg_types{"string", "std_msgs/String"};
};

template<>
struct ROS1DecodabilityTraits<std::chrono::nanoseconds> {
    static constexpr std::array<std::string_view, 2> msg_types{"duration", "std_msgs/Duration"};
};

template<>
struct ROS1DecodabilityTraits<std::chrono::steady_clock::time_point> {
    static constexpr std::array<std::string_view, 2> msg_types{"time", "std_msgs/Time"};
};

template<>
struct ROS1DecodabilityTraits<std::chrono::system_clock::time_point> {
    static constexpr std::array<std::string_view, 2> msg_types{"time", "std_msgs/Time"};
};

template<>
struct ROS1DecodabilityTraits<Eigen::Vector3d> {
    static constexpr std::array<std::string_view, 2> msg_types{"geometry_msgs/Point", "geometry_msgs/Vector3"};
};

template<>
struct ROS1DecodabilityTraits<Eigen::Quaterniond> {
    static constexpr std::array<std::string_view, 1> msg_types{"geometry_msgs/Quaternion"};
};

template<>
struct ROS1DecodabilityTraits<Eigen::Isometry3d> {
    static constexpr std::array<std::string_view, 3> msg_types{"geometry_msgs/Pose", "geometry_msgs/PoseWithCovariance",
            "geometry_msgs/Transform"};
};

template<>
struct ROS1DecodabilityTraits<ActuatorMeasurement> {
    static constexpr std::array<std::string_view, 1> msg_types{"series_elastic_actuator_msgs/SeActuatorReading"};
};

template<>
struct ROS1DecodabilityTraits<std::vector<ActuatorMeasurement>> {
    static constexpr std::array<std::string_view, 1> msg_types{"series_elastic_actuator_msgs/SeActuatorReadings"};
};

template<>
struct ROS1DecodabilityTraits<ContactClassifications> {
    static constexpr std::array<std::string_view, 1> msg_types{"anymal_msgs/AnymalState"};
};

template<>
struct ROS1DecodabilityTraits<ImuMeasurement<3>> {
    static constexpr std::array<std::string_view, 1> msg_types{"sensor_msgs/Imu"};
};

template<>
struct ROS1DecodabilityTraits<PoseMeasurement<3>> {
    static constexpr std::array<std::string_view, 4> msg_types{"geometry_msgs/PoseStamped",
            "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/TransformStamped", "nav_msgs/Odometry"};
};

template<>
struct ROS1DecodabilityTraits<std::vector<PoseMeasurement<3>>> {
    static constexpr std::array<std::string_view, 1> msg_types{"tf2_msgs/TFMessage"};
};

template<>
struct ROS1DecodabilityTraits<TemporalMeasurement> {
    static constexpr std::array<std::string_view, 1> msg_types{"std_msgs/Header"};
};

template<>
struct ROS1DecodabilityTraits<TemporalSpatialMeasurement> {
    static constexpr std::array<std::string_view, 1> msg_types{"std_msgs/Header"};
};

}

#include "sensorbox/impl/ros1_decoder.hpp"

#endif
