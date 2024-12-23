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

    /**
     * @brief Create a new pose measurement with a different child frame, as:
     *
     * \f[
     *      T_{F,N} = T_{F,C} * T_{C,N}
     * \f]
     *
     * @param new_child_frame new child frame \f$N\f$
     * @param T_C_N pose of new child frame in the current child frame \f$T_{C,N}\f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_child_frame(const std::string& new_child_frame, const Pose& T_C_N) const;

    /**
     * @brief Create a new pose measurement with a different frame, as:
     *
     * \f[
     *      T_{N,C} = T_{N,F} * T_{F,C}
     * \f]
     *
     * @param new_frame new frame \f$N\f$
     * @param T_N_F pose of current frame in the new frame \f$T_{N,F}\f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_frame(const std::string& new_frame, const Pose& T_N_F) const;

    /**
     * @brief Create a new pose measurement with different frames, as:
     *
     * \f[
     *      T_{NF,NC} = T_{NF,F} * T_{F,C} * T_{C,NC}
     * \f]
     *
     * @param new_frame new frame \f$NF\f$
     * @param new_child_frame new child frame \f$NC\f$
     * @param T_NF_F pose of current frame in the new frame \f$T_{NF,F}\f$
     * @param T_C_NC pose of new child frame in the current child frame \f$T_{C,N}\f$
     * @return PoseMeasurement
     */
    PoseMeasurement transform_to_new_frames(const std::string& new_frame, const std::string& new_child_frame,
            const Pose& T_NF_F, const Pose& T_C_NC) const;

private:
    Pose pose_;
};

}

#include "sensorbox/impl/pose.hpp"

#endif
