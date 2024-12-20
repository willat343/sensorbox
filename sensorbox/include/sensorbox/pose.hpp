#ifndef SENSORBOX_POSE_HPP
#define SENSORBOX_POSE_HPP

#include <Eigen/Geometry>
#include <string>

#include "sensorbox/unary.hpp"

namespace sensorbox {

template<int D_>
class PoseMeasurement : public UnaryMeasurement {
public:
    using Clock = UnaryMeasurement::Clock;
    using Duration = UnaryMeasurement::Duration;
    using Timestamp = UnaryMeasurement::Timestamp;
    static constexpr int D = D_;
    using Pose = Eigen::Transform<double, D, Eigen::Isometry>;

    explicit PoseMeasurement();

    /**
     * @brief Construct a Pose Measurement, of child_frame_ (C) in frame_ (F), i.e. \f$\mathbf{T}_{FC}\f$.
     *
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is made, e.g. map_frame
     * @param child_frame_ reference frame that has been measured, e.g. body_frame
     * @param pose_ pose of child_frame_ in frame_, \f$\mathbf{T}_{FC}\f$
     */
    explicit PoseMeasurement(const Timestamp& timestamp_, const std::string& frame_, const std::string& child_frame_,
            const Pose& pose_);

    const Pose& pose() const;

    Pose& pose();

private:
    Pose pose_;
};

}

#include "sensorbox/impl/pose.hpp"

#endif
