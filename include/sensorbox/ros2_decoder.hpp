#ifndef SENSORBOX_ROS2_DECODER_HPP
#define SENSORBOX_ROS2_DECODER_HPP

#include <Eigen/Core>
#include <array>
#include <cppbox/array.hpp>
#include <cppbox/bytes.hpp>
#include <cppbox/time.hpp>
#include <numeric>
#include <optional>
#include <span>
#include <string>

#include "sensorbox/imu.hpp"
#include "sensorbox/measurement.hpp"
#include "sensorbox/message_decoder.hpp"
#include "sensorbox/pose.hpp"
#include "sensorbox/position.hpp"

namespace sensorbox {

struct ROS2MessagesTypes {
    /**
     * @brief In ROS 2, (top-level) messages start with CDR header (assumed little-endian).
     *
     */
    static constexpr std::array<std::byte, 4> cdr_header{
            {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}}};

    struct fundamental {
        static constexpr auto sizes = std::to_array<MessageSize>({
                {"bool", sizeof(bool)},
                {"byte", sizeof(uint8_t)},
                {"char", sizeof(char)},
                {"int8", sizeof(int8_t)},
                {"uint8", sizeof(uint8_t)},
                {"int16", sizeof(int16_t)},
                {"int16", sizeof(int16_t)},
                {"uint16", sizeof(uint16_t)},
                {"int32", sizeof(int32_t)},
                {"uint32", sizeof(uint32_t)},
                {"int64", sizeof(int64_t)},
                {"uint64", sizeof(uint64_t)},
                {"float32", sizeof(float)},
                {"float64", sizeof(double)},
        });

        /**
         * @brief Compute the ROS 2 CDR aligned offset of a fundmental type of `size_` bytes based on the current offset
         * from the end of the CDR header.
         *
         * In ROS 2, CDR structs align fundamental types with leading padding to memory offsets that are multiples of
         * their own size.
         *
         * Hence:
         * - 1 byte: no alignment
         * - 2 bytes: starts on even byte offset
         * - 4 bytes: starts on multiple of 4 bytes
         * - 8 bytes: starts on multiple of 8 bytes
         *
         * There is no trailing padding.
         *
         * @param size_
         * @param offset
         * @return constexpr std::size_t
         */
        static constexpr std::size_t aligned_offset(const std::size_t size_, const std::size_t offset = 0) {
            // Since size_ is a power of 2, use bit-masking to compute the aligned offset efficiently
            assert(size_ == 0 || (size_ & (size_ - 1)) == 0);
            return size_ == 0 ? offset : (offset + (size_ - 1)) & ~(size_ - 1);
        }

        static constexpr std::size_t padding(const std::size_t size_, const std::size_t offset = 0) {
            // Since size_ is a power of 2, use bit-masking to compute the padding efficiently
            assert(size_ == 0 || (size_ & (size_ - 1)) == 0);
            return size_ == 0 ? 0 : (-offset) & (size_ - 1);
        }

        /**
         * @brief Compute the ROS 2 CDR size of a fundament type including its padding based on the current offset from
         * the end of the CDR header.
         *
         * See `std::size_t padding(const std::size_t, const std::size_t)` for more details.
         *
         * @param msg_type
         * @return constexpr std::size_t
         */
        static constexpr std::size_t size(const std::string_view msg_type, const std::size_t offset = 0) {
            const auto it = std::find_if(sizes.cbegin(), sizes.cend(),
                    [msg_type](const MessageSize& message_size) { return message_size.type == msg_type; });
            std::size_t unpadded_size = it != sizes.cend() ? it->size : 0;
            return padding(unpadded_size, offset) + unpadded_size;
        }
    };

    struct builtin_interfaces {
        static constexpr auto Time = std::to_array<MessageField>({
                {"int32", "sec"},
                {"uint32", "nanosec"},
        });
        static constexpr auto Duration = std::to_array<MessageField>({
                {"int32", "sec"},
                {"uint32", "nanosec"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"builtin_interfaces/Time", Time},
                {"builtin_interfaces/Duration", Duration},
        });

        static constexpr std::size_t size(const std::string_view msg_type, const std::size_t offset = 0) {
            const auto it = std::find_if(msg_types.cbegin(), msg_types.cend(),
                    [msg_type](const MessageType& message_type) { return message_type.type == msg_type; });
            return it == msg_types.cend() ? 0
                                          : std::accumulate(it->fields.begin(), it->fields.end(), 0,
                                                    [offset](std::size_t sum, const MessageField& item) {
                                                        return sum + fundamental::size(item.type, offset);
                                                    });
        }
    };

    struct std_msgs {
        static constexpr auto Bool = std::to_array<MessageField>({
                {"bool", "data"},
        });
        static constexpr auto Byte = std::to_array<MessageField>({
                {"byte", "data"},
        });
        static constexpr auto ByteMultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"byte[]", "data"},
        });
        static constexpr auto Char = std::to_array<MessageField>({
                {"char", "data"},
        });
        static constexpr auto ColorRGBA = std::to_array<MessageField>({
                {"float32", "r"},
                {"float32", "g"},
                {"float32", "b"},
                {"float32", "a"},
        });
        static constexpr auto Duration = std::to_array<MessageField>({
                {"duration", "data"},
        });
        static constexpr auto Empty = std::array<MessageField, 0>();
        static constexpr auto Float32 = std::to_array<MessageField>({
                {"float32", "data"},
        });
        static constexpr auto Float32MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"float32[]", "data"},
        });
        static constexpr auto Float64 = std::to_array<MessageField>({
                {"float64", "data"},
        });
        static constexpr auto Float64MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"float64[]", "data"},
        });
        // In ROS 2 `seq` was dropped and `stamp` changed type
        static constexpr auto Header = std::to_array<MessageField>({
                {"builtin_interfaces/Time", "stamp"},
                {"string", "frame_id"},
        });
        static constexpr auto Int16 = std::to_array<MessageField>({
                {"int16", "data"},
        });
        static constexpr auto Int16MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"int16[]", "data"},
        });
        static constexpr auto Int32 = std::to_array<MessageField>({
                {"int32", "data"},
        });
        static constexpr auto Int32MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"int32[]", "data"},
        });
        static constexpr auto Int64 = std::to_array<MessageField>({
                {"int64", "data"},
        });
        static constexpr auto Int64MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"int64[]", "data"},
        });
        static constexpr auto Int8 = std::to_array<MessageField>({
                {"int8", "data"},
        });
        static constexpr auto Int8MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"int8[]", "data"},
        });
        static constexpr auto MultiArrayDimension = std::to_array<MessageField>({
                {"string", "label"},
                {"uint32", "size"},
                {"uint32", "stride"},
        });
        static constexpr auto MultiArrayLayout = std::to_array<MessageField>({
                {"MultiArrayDimension[]", "dim"},
                {"uint32", "data_offset"},
        });
        static constexpr auto String = std::to_array<MessageField>({
                {"string", "data"},
        });
        static constexpr auto Time = std::to_array<MessageField>({
                {"time", "data"},
        });
        static constexpr auto UInt16 = std::to_array<MessageField>({
                {"uint16", "data"},
        });
        static constexpr auto UInt16MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"uint16[]", "data"},
        });
        static constexpr auto UInt32 = std::to_array<MessageField>({
                {"uint32", "data"},
        });
        static constexpr auto UInt32MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"uint32[]", "data"},
        });
        static constexpr auto UInt64 = std::to_array<MessageField>({
                {"uint64", "data"},
        });
        static constexpr auto UInt64MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"uint64[]", "data"},
        });
        static constexpr auto UInt8 = std::to_array<MessageField>({
                {"uint8", "data"},
        });
        static constexpr auto UInt8MultiArray = std::to_array<MessageField>({
                {"std_msgs/MultiArrayLayout", "layout"},
                {"uint8[]", "data"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"std_msgs/Bool", Bool},
                {"std_msgs/Byte", Byte},
                {"std_msgs/ByteMultiArray", ByteMultiArray},
                {"std_msgs/Char", Char},
                {"std_msgs/ColorRGBA", ColorRGBA},
                {"std_msgs/Duration", Duration},
                {"std_msgs/Empty", Empty},
                {"std_msgs/Float32", Float32},
                {"std_msgs/Float32MultiArray", Float32MultiArray},
                {"std_msgs/Float64", Float64},
                {"std_msgs/Float64MultiArray", Float64MultiArray},
                {"std_msgs/Header", Header},
                {"std_msgs/Int16", Int16},
                {"std_msgs/Int16MultiArray", Int16MultiArray},
                {"std_msgs/Int32", Int32},
                {"std_msgs/Int32MultiArray", Int32MultiArray},
                {"std_msgs/Int64", Int64},
                {"std_msgs/Int64MultiArray", Int64MultiArray},
                {"std_msgs/Int8", Int8},
                {"std_msgs/MultiArrayDimension", MultiArrayDimension},
                {"std_msgs/MultiArrayLayout", MultiArrayLayout},
                {"std_msgs/String", String},
                {"std_msgs/Time", Time},
                {"std_msgs/UInt16", UInt16},
                {"std_msgs/UInt16MultiArray", UInt16MultiArray},
                {"std_msgs/UInt32", UInt32},
                {"std_msgs/UInt32MultiArray", UInt32MultiArray},
                {"std_msgs/UInt64", UInt64},
                {"std_msgs/UInt64MultiArray", UInt64MultiArray},
                {"std_msgs/UInt8", UInt8},
                {"std_msgs/UInt8MultiArray", UInt8MultiArray},
        });
    };

    struct geometry_msgs {
        static constexpr auto Accel = std::to_array<MessageField>({
                {"geometry_msgs/Vector3", "linear"},
                {"geometry_msgs/Vector3", "angular"},
        });
        static constexpr auto AccelStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Accel", "accel"},
        });
        static constexpr auto AccelWithCovariance = std::to_array<MessageField>({
                {"geometry_msgs/Accel", "accel"},
                {"float64[36]", "covariance"},
        });
        static constexpr auto AccelWithCovarianceStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/AccelWithCovariance", "accel"},
        });
        static constexpr auto Inertia = std::to_array<MessageField>({
                {"float63", "m"},
                {"geometry_msgs/Vector3", "com"},
                {"float64", "ixx"},
                {"float64", "ixy"},
                {"float64", "ixz"},
                {"float64", "iyy"},
                {"float64", "iyz"},
                {"float64", "izz"},
        });
        static constexpr auto InertiaStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Inertia", "inertia"},
        });
        static constexpr auto Point = std::to_array<MessageField>({
                {"float64", "x"},
                {"float64", "y"},
                {"float64", "z"},
        });
        static constexpr auto Point32 = std::to_array<MessageField>({
                {"float32", "x"},
                {"float32", "y"},
                {"float32", "z"},
        });
        static constexpr auto PointStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Point", "point"},
        });
        static constexpr auto Polygon = std::to_array<MessageField>({
                {"geometry_msgs/Point32[]", "points"},
        });
        // ROS 2 introduced PolygonInstance
        static constexpr auto PolygonInstance = std::to_array<MessageField>({
                {"geometry_msgs/Polygon", "polygon"},
                {"int64", "id"},
        });
        // ROS 2 introduced PolygonInstanceStamped
        static constexpr auto PolygonInstanceStamped = std::to_array<MessageField>({
                {"stdd_msgs/Header", "header"},
                {"geometry_msgs/PolygonInstance", "polygon"},
        });
        static constexpr auto PolygonStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Polygon", "polygon"},
        });
        static constexpr auto Pose = std::to_array<MessageField>({
                {"geometry_msgs/Point", "position"},
                {"geometry_msgs/Quaternion", "orientation"},
        });
        static constexpr auto Pose2D = std::to_array<MessageField>({
                {"float64", "x"},
                {"float64", "y"},
                {"float64", "theta"},
        });
        static constexpr auto PoseArray = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Pose[]", "poses"},
        });
        static constexpr auto PoseWithCovariance = std::to_array<MessageField>({
                {"geometry_msgs/Pose", "pose"},
                {"float64[36]", "covariance"},
        });
        static constexpr auto PoseWithCovarianceStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/PoseWithCovariance", "pose"},
        });
        static constexpr auto Quaternion = std::to_array<MessageField>({
                {"float64", "x"},
                {"float64", "y"},
                {"float64", "z"},
                {"float64", "w"},
        });
        static constexpr auto QuaternionStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Quaternion", "quaternion"},
        });
        static constexpr auto Transform = std::to_array<MessageField>({
                {"geometry_msgs/Vector3", "translation"},
                {"geometry_msgs/Quaternion", "rotation"},
        });
        static constexpr auto TransformStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Transform", "transform"},
        });
        static constexpr auto Twist = std::to_array<MessageField>({
                {"geometry_msgs/Vector3", "linear"},
                {"geometry_msgs/Vector3", "angular"},
        });
        static constexpr auto TwistStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Twist", "twist"},
        });
        static constexpr auto TwistWithCovariance = std::to_array<MessageField>({
                {"geometry_msgs/Twist", "twist"},
                {"float64[36]", "covariance"},
        });
        static constexpr auto TwistWithCovarianceStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/TwistWithCovariance", "twist"},
        });
        static constexpr auto Vector3 = std::to_array<MessageField>({
                {"float64", "x"},
                {"float64", "y"},
                {"float64", "z"},
        });
        static constexpr auto Vector3Stamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Vector3", "vector"},
        });
        // ROS 2 introduced VelocityStamped
        static constexpr auto VelocityStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"string", "body_frame_id"},
                {"string", "reference_frame_id"},
                {"geometry_msgs/Twist", "velocity"},
        });
        static constexpr auto Wrench = std::to_array<MessageField>({
                {"geometry_msgs/Vector3", "force"},
                {"geometry_msgs/Vector3", "torque"},
        });
        static constexpr auto WrenchStamped = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Wrench", "wrench"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"geometry_msgs/Accel", Accel},
                {"geometry_msgs/AccelStamped", AccelStamped},
                {"geometry_msgs/AccelWithCovariance", AccelWithCovariance},
                {"geometry_msgs/AccelWithCovarianceStamped", AccelWithCovarianceStamped},
                {"geometry_msgs/Inertia", Inertia},
                {"geometry_msgs/InertiaStamped", InertiaStamped},
                {"geometry_msgs/Point", Point},
                {"geometry_msgs/Point32", Point32},
                {"geometry_msgs/PointStamped", PointStamped},
                {"geometry_msgs/Polygon", Polygon},
                {"geometry_msgs/PolygonInstance", PolygonInstance},
                {"geometry_msgs/PolygonInstanceStamped", PolygonInstanceStamped},
                {"geometry_msgs/PolygonStamped", PolygonStamped},
                {"geometry_msgs/Pose", Pose},
                {"geometry_msgs/Pose2D", Pose2D},
                {"geometry_msgs/PoseArray", PoseArray},
                {"geometry_msgs/PoseWithCovariance", PoseWithCovariance},
                {"geometry_msgs/PoseWithCovarianceStamped", PoseWithCovarianceStamped},
                {"geometry_msgs/Quaternion", Quaternion},
                {"geometry_msgs/QuaternionStamped", QuaternionStamped},
                {"geometry_msgs/Transform", Transform},
                {"geometry_msgs/TransformStamped", TransformStamped},
                {"geometry_msgs/Twist", Twist},
                {"geometry_msgs/TwistStamped", TwistStamped},
                {"geometry_msgs/TwistWithCovariance", TwistWithCovariance},
                {"geometry_msgs/TwistWithCovarianceStamped", TwistWithCovarianceStamped},
                {"geometry_msgs/Vector3", Vector3},
                {"geometry_msgs/Vector3Stamped", Vector3Stamped},
                {"geometry_msgs/VelocityStamped", VelocityStamped},
                {"geometry_msgs/Wrench", Wrench},
                {"geometry_msgs/WrenchStamped", WrenchStamped},
        });
    };

    struct nav_msgs {
        // ROS 2 introduced Goals
        static constexpr auto Goals = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/PoseStamped[]", "goals"},
        });
        static constexpr auto GridCells = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float32", "cell_width"},
                {"float32", "cell_height"},
                {"geometry_msgs/Point[]", "cells"},
        });
        // In ROS 2 `map_load_time` changed type
        static constexpr auto MapMetaData = std::to_array<MessageField>({
                {"builtin_interfaces/Time", "map_load_time"},
                {"float32", "resolution"},
                {"uint32", "width"},
                {"uint32", "height"},
                {"geometry_msgs/Pose", "origin"},
        });
        static constexpr auto OccupancyGrid = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"MapMetaData", "info"},
                {"int8[]", "data"},
        });
        static constexpr auto Odometry = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"string", "child_frame_id"},
                {"geometry_msgs/PoseWithCovariance", "pose"},
                {"geometry_msgs/TwistWithCovariance", "twist"},
        });
        static constexpr auto Path = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/PoseStamped[]", "poses"},
        });
        // ROS 2 introduced Trajectory
        static constexpr auto Trajectory = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"nav_msgs/TrajectoryPoint[]", "points"},
        });
        // ROS 2 introduced TrajectoryPoint
        static constexpr auto TrajectoryPoint = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Pose", "pose"},
                {"geometry_msgs/Twist", "velocity"},
                {"geometry_msgs/Accel", "acceleration"},
                {"geometry_msgs/Wrench", "effort"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"geometry_msgs/GridCells", GridCells},
                {"nav_msgs/MapMetaData", MapMetaData},
                {"nav_msgs/OccupancyGrid", OccupancyGrid},
                {"nav_msgs/Odometry", Odometry},
                {"nav_msgs/Path", Path},
        });
    };

    struct sensor_msgs {
        struct battery_state {
            // Power supply status constants
            static constexpr uint8_t POWER_SUPPLY_STATUS_UNKNOWN = 0;
            static constexpr uint8_t POWER_SUPPLY_STATUS_CHARGING = 1;
            static constexpr uint8_t POWER_SUPPLY_STATUS_DISCHARGING = 2;
            static constexpr uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
            static constexpr uint8_t POWER_SUPPLY_STATUS_FULL = 4;

            // Power supply health constants
            static constexpr uint8_t POWER_SUPPLY_HEALTH_UNKNOWN = 0;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_GOOD = 1;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERHEAT = 2;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_DEAD = 3;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_COLD = 6;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
            static constexpr uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8;

            // Power supply technology (chemistry) constants
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH = 1;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LION = 2;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO = 3;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE = 4;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NICD = 5;
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN = 6;
            // Added in ROS 2
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_TERNARY = 7;
            // Added in ROS 2
            static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_VRLA = 8;
        };
        static constexpr auto BatteryState = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float32", "voltage"},
                {"float32", "temperature"},
                {"float32", "current"},
                {"float32", "charge"},
                {"float32", "capacity"},
                {"float32", "design_capacity"},
                {"float32", "percentage"},
                {"uint8", "power_supply_status"},
                {"uint8", "power_supply_health"},
                {"uint8", "power_supply_technology"},
                {"bool", "present"},
                {"float32[]", "cell_voltage"},
                {"float32[]", "cell_temperature"},
                {"string", "location"},
                {"string", "serial_number"},
        });
        // ROS 2 changed naming convention of fields from D, K, R, and P to d, k, r, and p respectively
        static constexpr auto CameraInfo = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"uint32", "height"},
                {"uint32", "width"},
                {"string", "distortion_model"},
                {"float64[]", "d"},
                {"float64[9]", "k"},
                {"float64[9]", "r"},
                {"float64[12]", "p"},
                {"uint32", "binning_x"},
                {"uint32", "binning_y"},
                {"sensor_msgs/RegionOfInterest", "roi"},
        });
        static constexpr auto ChannelFloat32 = std::to_array<MessageField>({
                {"string", "name"},
                {"float32[]", "values"},
        });
        static constexpr auto CompressedImage = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"string", "format"},
                {"uint8[]", "data"},
        });
        static constexpr auto FluidPressure = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float64", "fluid_pressure"},
                {"float64", "variance"},
        });
        static constexpr auto Illuminance = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float64", "illuminance"},
                {"float64", "variance"},
        });
        static constexpr auto Image = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"uint32", "height"},
                {"uint32", "width"},
                {"string", "encoding"},
                {"uint8", "is_bigendian"},
                {"uint32", "step"},
                {"uint8[]", "data"},
        });
        static constexpr auto Imu = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Quaternion", "orientation"},
                {"float64[9]", "orientation_covariance"},
                {"geometry_msgs/Vector3", "angular_velocity"},
                {"float64[9]", "angular_velocity_covariance"},
                {"geometry_msgs/Vector3", "linear_acceleration"},
                {"float64[9]", "linear_acceleration_covariance"},
        });
        static constexpr auto JointState = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"string[]", "name"},
                {"float64[]", "position"},
                {"float64[]", "velocity"},
                {"float64[]", "effort"},
        });
        static constexpr auto Joy = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float32[]", "axes"},
                {"int32[]", "buttons"},
        });
        struct joy_feedback {
            static constexpr uint8_t TYPE_LED = 0;
            static constexpr uint8_t TYPE_RUMBLE = 1;
            static constexpr uint8_t TYPE_BUZZER = 2;
        };
        static constexpr auto JoyFeedback = std::to_array<MessageField>({
                {"uint8", "type"},
                {"uint8", "id"},
                {"float32", "intensity"},
        });
        static constexpr auto JoyFeedbackArray = std::to_array<MessageField>({
                {"sensor_msgs/JoyFeedback[]", "array"},
        });
        static constexpr auto LaserEcho = std::to_array<MessageField>({
                {"float32[]", "echoes"},
        });
        static constexpr auto LaserScan = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float32", "angle_min"},
                {"float32", "angle_max"},
                {"float32", "angle_increment"},
                {"float32", "time_increment"},
                {"float32", "scan_time"},
                {"float32", "range_min"},
                {"float32", "range_max"},
                {"float32[]", "ranges"},
                {"float32[]", "intensities"},
        });
        static constexpr auto MagneticField = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Vector3", "magnetic_field"},
                {"float[64]", "magnetic_field_covariance"},
        });
        static constexpr auto MultiDOFJointState = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"string[]", "joint_names"},
                {"geometry_msgs/Transform[]", "transforms"},
                {"geometry_msgs/Twist[]", "twist"},
                {"geometry_msgs/Wrench[]", "wrench"},
        });
        static constexpr auto MultiEchoLaserScan = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float32", "angle_min"},
                {"float32", "angle_max"},
                {"float32", "angle_increment"},
                {"float32", "time_increment"},
                {"float32", "scan_time"},
                {"float32", "range_min"},
                {"float32", "range_max"},
                {"sensor_msgs/LaserEcho[]", "ranges"},
                {"sensor_msgs/LaserEcho[]", "intensities"},
        });
        struct nav_sat_fix {
            static constexpr uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
            static constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
            static constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
            static constexpr uint8_t COVARIANCE_TYPE_KNOWN = 3;
        };
        static constexpr auto NavSatFix = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"sensor_msgs/NavSatStatus", "status"},
                {"float64", "latitude"},
                {"float64", "longitude"},
                {"float64", "altitude"},
                {"float64[9]", "position_covariance"},
                {"uint8", "position_covariance_type"},
        });
        struct nav_sat_status {
            // Whether to output an augmented fix is determined by both the fix type and the last time differential
            // corrections were received. A fix is valid when status >= STATUS_FIX.
            static constexpr int8_t STATUS_NO_FIX = -1;
            static constexpr int8_t STATUS_FIX = 0;
            static constexpr int8_t STATUS_SBAS_FIX = 1;
            static constexpr int8_t STATUS_GBAS_FIX = 2;
            // Bits defining which Global Navigation Satellite System signals were used by the receiver.
            static constexpr uint16_t SERVICE_GPS = 1;
            static constexpr uint16_t SERVICE_GLONASS = 2;
            static constexpr uint16_t SERVICE_COMPASS = 4;
            static constexpr uint16_t SERVICE_GALILEO = 8;
        };
        static constexpr auto NavSatStatus = std::to_array<MessageField>({
                {"int8", "status"},
                {"uint16", "service"},
        });
        static constexpr auto PointCloud = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"geometry_msgs/Point32[]", "points"},
                {"sensor_msgs/ChannelFloat32[]", "channels"},
        });
        static constexpr auto PointCloud2 = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"uint32", "height"},
                {"uint32", "width"},
                {"sensor_msgs/PointField[]", "fields"},
                {"bool", "is_bigendian"},
                {"uint32", "point_step"},
                {"uint32", "row_step"},
                {"uint8[]", "data"},
                {"bool", "is_dense"},
        });
        struct point_cloud_2 {
            static constexpr uint8_t INT8 = 1;
            static constexpr uint8_t UINT8 = 2;
            static constexpr uint8_t INT16 = 3;
            static constexpr uint8_t UINT16 = 4;
            static constexpr uint8_t INT32 = 5;
            static constexpr uint8_t UINT32 = 6;
            static constexpr uint8_t FLOAT32 = 7;
            static constexpr uint8_t FLOAT64 = 8;
        };
        static constexpr auto PointField = std::to_array<MessageField>({
                {"string", "name"},
                {"uint32", "offset"},
                {"uint8", "datatype"},
                {"uint32", "count"},
        });
        struct range {
            static constexpr uint8_t ULTRASOUND = 0;
            static constexpr uint8_t INFRARED = 1;
        };
        static constexpr auto Range = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"uint8", "radiation_type"},
                {"float32", "field_of_view"},
                {"float32", "min_range"},
                {"float32", "max_range"},
                {"float32", "range"},
        });
        static constexpr auto RegionOfInterest = std::to_array<MessageField>({
                {"uint32", "x_offset"},
                {"uint32", "y_offset"},
                {"uint32", "height"},
                {"uint32", "width"},
                {"bool", "do_rectify"},
        });
        static constexpr auto RelativeHumidity = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float64", "relative_humidity"},
                {"float64", "variance"},
        });
        static constexpr auto Temperature = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"float64", "temperature"},
                {"float64", "variance"},
        });
        // ROS 2 changed `time_ref` type
        static constexpr auto TimeReference = std::to_array<MessageField>({
                {"std_msgs/Header", "header"},
                {"builtin_interfaces/Time", "time_ref"},
                {"string", "source"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"sensor_msgs/BatteryState", BatteryState},
                {"sensor_msgs/CameraInfo", CameraInfo},
                {"sensor_msgs/ChannelFloat32", ChannelFloat32},
                {"sensor_msgs/CompressedImage", CompressedImage},
                {"sensor_msgs/FluidPressure", FluidPressure},
                {"sensor_msgs/Illuminance", Illuminance},
                {"sensor_msgs/Image", Image},
                {"sensor_msgs/Imu", Imu},
                {"sensor_msgs/JointState", JointState},
                {"sensor_msgs/Joy", Joy},
                {"sensor_msgs/JoyFeedback", JoyFeedback},
                {"sensor_msgs/JoyFeedbackArray", JoyFeedbackArray},
                {"sensor_msgs/LaserEcho", LaserEcho},
                {"sensor_msgs/LaserScan", LaserScan},
                {"sensor_msgs/MagneticField", MagneticField},
                {"sensor_msgs/MultiDOFJointState", MultiDOFJointState},
                {"sensor_msgs/MultiEchoLaserScan", MultiEchoLaserScan},
                {"sensor_msgs/NavSatFix", NavSatFix},
                {"sensor_msgs/NavSatStatus", NavSatStatus},
                {"sensor_msgs/PointCloud", PointCloud},
                {"sensor_msgs/PointCloud2", PointCloud2},
                {"sensor_msgs/PointField", PointField},
                {"sensor_msgs/Range", Range},
                {"sensor_msgs/RegionOfInterest", RegionOfInterest},
                {"sensor_msgs/RelativeHumidity", RelativeHumidity},
                {"sensor_msgs/Temperature", Temperature},
                {"sensor_msgs/TimeReference", TimeReference},
        });
    };

    struct tf2_msgs {
        struct tf2_error {
            static constexpr uint8_t NO_ERROR = 0;
            static constexpr uint8_t LOOKUP_ERROR = 1;
            static constexpr uint8_t CONNECTIVITY_ERROR = 2;
            static constexpr uint8_t EXTRAPOLATION_ERROR = 3;
            static constexpr uint8_t INVALID_ARGUMENT_ERROR = 4;
            static constexpr uint8_t TIMEOUT_ERROR = 5;
            static constexpr uint8_t TRANSFORM_ERROR = 6;
        };
        static constexpr auto TF2Error = std::to_array<MessageField>({
                {"uint8", "error"},
                {"string", "error_string"},
        });
        static constexpr auto TFMessage = std::to_array<MessageField>({
                {"geometry_msgs/TransformStamped[]", "transforms"},
        });

        static constexpr auto msg_types = std::to_array<MessageType>({
                {"tf2_msgs/TF2Error", TF2Error},
                {"tf2_msgs/TFMessage", TFMessage},
        });
    };

    static constexpr auto msg_types = cppbox::merge(std_msgs::msg_types, geometry_msgs::msg_types, nav_msgs::msg_types,
            sensor_msgs::msg_types, tf2_msgs::msg_types);

    static constexpr std::string_view starts_with(const std::string_view msg_type) {
        return message_starts_with(msg_types, msg_type);
    }

    static std::string remove_internal_msg_substring(const std::string_view msg_type) {
        constexpr std::string_view target = "msg/";
        if (std::size_t pos = msg_type.find(target); pos != std::string_view::npos) {
            return std::string(msg_type.substr(0, pos)) + std::string(msg_type.substr(pos + target.size()));
        }
        return std::string(msg_type);
    }
};

struct ROS2Conversions {
    template<typename T>
    static constexpr auto decodable_msg_types() {
        if constexpr (std::same_as<T, std::string>) {
            return std::to_array<std::string_view>({"string", "std_msgs/String"});
        } else if constexpr (cppbox::IsDuration<T>) {
            return std::to_array<std::string_view>({"duration", "std_msgs/Duration"});
        } else if constexpr (cppbox::IsTimePoint<T>) {
            return std::to_array<std::string_view>({"time", "std_msgs/Time"});
        } else if constexpr (std::same_as<T, Eigen::Vector3d>) {
            return std::to_array<std::string_view>({"geometry_msgs/Point", "geometry_msgs/Vector3"});
        } else if constexpr (std::same_as<T, Eigen::Quaterniond>) {
            return std::to_array<std::string_view>({"geometry_msgs/Quaternion"});
        } else if constexpr (std::same_as<T, Eigen::Isometry3d>) {
            return std::to_array<std::string_view>(
                    {"geometry_msgs/Pose", "geometry_msgs/PoseWithCovariance", "geometry_msgs/Transform"});
        } else if constexpr (std::same_as<T, ImuMeasurement<3>>) {
            return std::to_array<std::string_view>({"sensor_msgs/Imu"});
        } else if constexpr (std::same_as<T, PoseMeasurement<3>>) {
            return std::to_array<std::string_view>({"geometry_msgs/PoseStamped",
                    "geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/TransformStamped", "nav_msgs/Odometry"});
        } else if constexpr (std::same_as<T, PositionMeasurement<3>>) {
            return std::to_array<std::string_view>({"geometry_msgs/PointStamped", "geometry_msgs/Vector3Stamped",
                    "geometry_msgs/PoseStamped", "geometry_msgs/PoseWithCovarianceStamped",
                    "geometry_msgs/TransformStamped", "nav_msgs/Odometry"});
        } else if constexpr (std::same_as<T, std::vector<PoseMeasurement<3>>>) {
            return std::to_array<std::string_view>({"nav_msgs/Path", "tf2_msgs/TFMessage"});
        } else if constexpr (std::same_as<T, PoseMeasurements<3>>) {
            return std::to_array<std::string_view>({"nav_msgs/Path", "tf2_msgs/TFMessage"});
        } else if constexpr (std::same_as<T, TemporalMeasurement>) {
            return std::to_array<std::string_view>({"std_msgs/Header"});
        } else if constexpr (std::same_as<T, TemporalSpatialMeasurement>) {
            return std::to_array<std::string_view>({"std_msgs/Header"});
        } else {
            return std::array<std::string_view, 0>();
        }
    }
};

class ROS2BytesDecoder : public MessageDecoder<ROS2MessagesTypes, ROS2Conversions> {
public:
    /**
     * @brief Construct a new ROS 2 decoder.
     *
     * @param bytes_
     * @param size_
     * @param msg_type_
     */
    explicit ROS2BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_);

    /**
     * @brief Decode all bytes to a T object.
     *
     * @tparam T
     * @return T
     */
    template<typename T>
    T decode();

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
    void decode_optional_to(std::optional<T>& out);

    /**
     * @brief Decode all bytes to an optional T object.
     *
     * @tparam T
     * @return std::optional<T>
     */
    template<typename T>
    std::optional<T> decode_optional();

    /**
     * @brief Ignore a message of known type.
     *
     * @param msg_type
     * @param num_ignore number of messages to ignore (e.g., if a fixed-size array)
     */
    void ignore(const std::string_view msg_type, const std::size_t num_ignore = 1);

    /**
     * @brief Ignore a fundamental type. The user should not directly use the base class ignore functions because
     * padding must be considered.
     *
     * @tparam T
     * @param num_ignore number of messages to ignore (e.g., if a fixed-size array)
     */
    template<typename T>
        requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
    void ignore(const std::size_t num_ignore = 1);

    /**
     * @brief Read data to `T` without changing the internal offsets.
     *
     * @tparam T
     * @param extra_offset
     * @return T
     */
    template<typename T>
    T peak(const std::size_t extra_offset = 0);

    /**
     * @brief Read data to `T`.
     *
     * @tparam T
     * @return T
     */
    template<typename T>
    T read();

    /**
     * @brief Read data to `std::optional<T>`.
     *
     * @tparam T
     * @return std::optional<T>
     */
    template<typename T>
    std::optional<T> read_optional();

    /**
     * @brief Read data to `std::optional<T>`.
     *
     * @tparam T
     * @param out
     */
    template<typename T>
    void read_optional_to(std::optional<T>& out);

    /**
     * @brief Read fundamental data to `T`.
     *
     * @tparam T
     */
    template<typename T>
        requires(std::is_trivially_copyable_v<T> && !cppbox::IsTimePoint<T> && !cppbox::IsDuration<T>)
    void read_to(T& out);

    /**
     * @brief Read string data.
     *
     * @param out
     */
    void read_to(std::string& out);

    /**
     * @brief Read duration data.
     *
     * @tparam T
     * @param out
     */
    template<cppbox::IsDuration T>
    void read_to(T& out);

    /**
     * @brief Read time data.
     *
     * @tparam T
     * @param out
     */
    template<cppbox::IsTimePoint T>
    void read_to(T& out);

    void read_to(Eigen::Ref<Eigen::Vector3d> out);

    void read_to(Eigen::Quaterniond& out);

    void read_to(Eigen::Isometry3d& out);

    void read_to(ImuMeasurement<3>& out);

    void read_to(PoseMeasurement<3>& out);

    void read_to(std::vector<PoseMeasurement<3>>& out);

    void read_to(PoseMeasurements<3>& out);

    void read_to(PositionMeasurement<3>& out);

    void read_to(TemporalMeasurement& out);

    void read_to(TemporalSpatialMeasurement& out);

protected:
    explicit ROS2BytesDecoder(const std::byte* bytes_, const std::size_t size_, const std::string& msg_type_,
            ROS2BytesDecoder* parent_decoder_);

    ROS2BytesDecoder create_internal_decoder(const std::string& internal_msg_type);

    template<typename T>
    T decode_internal(const std::string& internal_msg_type);

    template<typename T>
    void decode_internal_to(const std::string& internal_msg_type, T& out);

    template<typename T>
    std::vector<T> decode_vector(const std::string& vector_msg_type);

    template<typename T>
    void decode_vector_to(const std::string& vector_msg_type, std::vector<T>& out);

    std::size_t internal_msg_size(const std::string_view internal_msg_type, std::size_t extra_offset = 0);
};

}

#include "sensorbox/impl/ros2_decoder.hpp"

#endif
