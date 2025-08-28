#ifndef SENSORBOX_MEASUREMENT_HPP
#define SENSORBOX_MEASUREMENT_HPP

#include <chrono>
#include <cppbox/enum.hpp>

namespace sensorbox {

CREATE_SMART_ENUM(MeasurementType, ACTUATOR_MEASUREMENT, ANGLE, ANGULAR_VELOCITY, CONTACT_CLASSIFICATIONS, CURRENT,
        IMU_MEASUREMENT, LINEAR_ACCELERATION, POSE, POSE_TWIST)

class TemporalMeasurement {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::nanoseconds;
    using Timestamp = std::chrono::time_point<Clock, Duration>;

    /**
     * @brief Construct a temporal measurement. The timestamp is initialised to zero.
     *
     */
    explicit TemporalMeasurement();

    /**
     * @brief Construct a temporal measurement.
     *
     * @param timestamp_ timestamp
     */
    explicit TemporalMeasurement(const Timestamp& timestamp_);

    const Timestamp& timestamp() const;

    Timestamp& timestamp();

private:
    /**
     * @brief Timestamp of the measurement.
     *
     */
    Timestamp timestamp_;
};

class TemporalSpatialMeasurement : public TemporalMeasurement {
public:
    using typename TemporalMeasurement::Clock;
    using typename TemporalMeasurement::Duration;
    using typename TemporalMeasurement::Timestamp;

    /**
     * @brief Construct a temporal-spatial measurement. The timestamp is initialised to zero and the frame is left
     * empty.
     *
     */
    explicit TemporalSpatialMeasurement();

    /**
     * @brief Construct a temporal measurement.
     *
     * @param timestamp_ timestamp
     */
    explicit TemporalSpatialMeasurement(const Timestamp& timestamp_, const std::string& frame_);

    const std::string& frame() const;

    std::string& frame();

private:
    /**
     * @brief Spatial frame of the measurement.
     *
     */
    std::string frame_;
};

class TemporalSpatialRelationalMeasurement : public TemporalSpatialMeasurement {
public:
    using typename TemporalSpatialMeasurement::Clock;
    using typename TemporalSpatialMeasurement::Duration;
    using typename TemporalSpatialMeasurement::Timestamp;

    /**
     * @brief Construct a temporal-spatial relational measurement. The timestamp is initialised to zero and the frames
     * is left empty.
     *
     */
    explicit TemporalSpatialRelationalMeasurement();

    /**
     * @brief Construct a temporal measurement.
     *
     * @param timestamp_ timestamp
     */
    explicit TemporalSpatialRelationalMeasurement(const Timestamp& timestamp_, const std::string& frame_,
            const std::string& child_frame_);

    const std::string& child_frame() const;

    std::string& child_frame();

private:
    /**
     * @brief Spatial childe frame of the relational measurement.
     *
     */
    std::string child_frame_;
};

}

#include "sensorbox/impl/measurement.hpp"

#endif
