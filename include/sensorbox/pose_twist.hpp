#ifndef SENSORBOX_POSE_TWIST_HPP
#define SENSORBOX_POSE_TWIST_HPP

#include <Eigen/Geometry>
#include <string>

#include "sensorbox/unary.hpp"

namespace sensorbox {

template<int D_>
class PoseTwistMeasurement : public UnaryMeasurement {
public:
    using Clock = UnaryMeasurement::Clock;
    using Duration = UnaryMeasurement::Duration;
    using Timestamp = UnaryMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;
    static constexpr int PoseDoF = D * (D + 1) / 2;
    using Twist = Eigen::Matrix<double, PoseDoF, 1>;

    explicit PoseTwistMeasurement();

    /**
     * @brief Construct a Pose-Twist Measurement.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$\mathbf{T}_{FC}\f$
     * @param twist_ twist of child_frame_ in child_frame_
     */
    explicit PoseTwistMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_, const Pose& pose_, const Twist& twist_);

    const std::string& child_frame() const;

    std::string& child_frame();

    const Pose& pose() const;

    Pose& pose();

    const Twist& twist() const;

    Twist& twist();

private:
    std::string child_frame_;
    Pose pose_;
    Twist twist_;
};

}

#include "sensorbox/impl/pose_twist.hpp"

#endif
