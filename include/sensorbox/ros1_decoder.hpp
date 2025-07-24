#ifndef SENSORBOX_ROS1_DECODER_HPP
#define SENSORBOX_ROS1_DECODER_HPP

#include <array>
#include <chrono>
#include <cppbox/bytes.hpp>
#include <string>

#include "sensorbox/pose.hpp"
#include "sensorbox/unary.hpp"

namespace sensorbox {

class ROS1BytesDecoder : public cppbox::BytesDecoder {
public:
    explicit ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_);

    ROS1BytesDecoder create_internal_decoder(const std::string& internal_msg_type);

    /**
     * @brief Decode all bytes to a T object.
     *
     * @tparam T
     * @return T
     */
    template<typename T>
    T decode_to();

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    void ignore(const std::size_t num_ignore = 1);

    template<typename T>
        requires(std::is_same_v<T, std::string>)
    void ignore(const std::size_t num_ignore = 1);

    void ignore(const std::string& msg_type);

    /**
     * @brief Check if there is support for decoding `msg_type`.
     *
     * @param msg_type
     * @return true
     * @return false
     */
    static constexpr bool is_decodable(const std::string_view& msg_type);

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
     * @brief Check if `msg_type` is decodable to type T.
     *
     * @tparam T
     * @param msg_type
     * @return true
     * @return false
     */
    template<typename T>
    static constexpr bool is_decodable_to(const std::string_view& msg_type);

    /**
     * @brief Check if `msg_type()` is decodable to type T.
     *
     * @tparam T
     * @return true
     * @return false
     */
    template<typename T>
    bool is_decodable_to();

    const std::string& msg_type() const;

    template<typename T>
        requires(std::is_trivially_copyable_v<T>)
    T peak(const std::size_t extra_offset = 0) const;

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

    void read_to(std::string& out);

    void read_to(std::chrono::nanoseconds& out);

    void read_to(std::chrono::steady_clock::time_point& out);

    void read_to(std::chrono::system_clock::time_point& out);

    void read_to(Eigen::Ref<Eigen::Vector3d> out);

    void read_to(Eigen::Quaterniond& out);

    void read_to(Eigen::Isometry3d& out);

    void read_to(UnaryMeasurement& out);

    void read_to(PoseMeasurement<3>& out);

protected:
    explicit ROS1BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
            ROS1BytesDecoder* parent_decoder_);

    std::size_t internal_msg_size(const std::string& internal_msg_type, std::size_t offset) const;

private:
    const std::string msg_type_;
};

template<typename T>
struct ROS1DecodabilityTraits {
    static constexpr std::array<std::string_view, 0> msg_types{};
};

template<>
struct ROS1DecodabilityTraits<std::string> {
    static constexpr std::array<std::string_view, 1> msg_types{"string"};
};

template<>
struct ROS1DecodabilityTraits<std::chrono::steady_clock::time_point> {
    static constexpr std::array<std::string_view, 1> msg_types{"time"};
};

template<>
struct ROS1DecodabilityTraits<std::chrono::system_clock::time_point> {
    static constexpr std::array<std::string_view, 1> msg_types{"time"};
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
struct ROS1DecodabilityTraits<UnaryMeasurement> {
    static constexpr std::array<std::string_view, 1> msg_types{"std_msgs/Header"};
};

template<>
struct ROS1DecodabilityTraits<PoseMeasurement<3>> {
    static constexpr std::array<std::string_view, 4> msg_types{"geometry_msgs/PoseStamped",
            "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/TransformStamped", "nav_msgs/Odometry"};
};

}

#include "sensorbox/impl/ros1_decoder.hpp"

#endif
