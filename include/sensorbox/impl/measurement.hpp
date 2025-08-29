#ifndef SENSORBOX_IMPL_MEASUREMENT_HPP
#define SENSORBOX_IMPL_MEASUREMENT_HPP

#include <utility>

#include "sensorbox/measurement.hpp"

namespace sensorbox {

inline TemporalMeasurement::TemporalMeasurement() : TemporalMeasurement(Timestamp{Duration::zero()}) {}

inline TemporalMeasurement::TemporalMeasurement(const Timestamp& timestamp_) : timestamp_(timestamp_) {}

inline auto TemporalMeasurement::timestamp() const -> const Timestamp& {
    return timestamp_;
}

inline auto TemporalMeasurement::timestamp() -> Timestamp& {
    return const_cast<Timestamp&>(std::as_const(*this).timestamp());
}

inline TemporalSpatialMeasurement::TemporalSpatialMeasurement()
    : TemporalSpatialMeasurement(Timestamp(Duration::zero()), std::string()) {}

inline TemporalSpatialMeasurement::TemporalSpatialMeasurement(const Timestamp& timestamp_, const std::string& frame_)
    : TemporalMeasurement(timestamp_), frame_(frame_) {}

inline const std::string& TemporalSpatialMeasurement::frame() const {
    return frame_;
}

inline std::string& TemporalSpatialMeasurement::frame() {
    return const_cast<std::string&>(std::as_const(*this).frame());
}

inline TemporalSpatialRelationalMeasurement::TemporalSpatialRelationalMeasurement()
    : TemporalSpatialRelationalMeasurement(Timestamp(Duration::zero()), std::string(), std::string()) {}

inline TemporalSpatialRelationalMeasurement::TemporalSpatialRelationalMeasurement(const Timestamp& timestamp_,
        const std::string& frame_, const std::string& child_frame_)
    : TemporalSpatialMeasurement(timestamp_, frame_), child_frame_(child_frame_) {}

inline const std::string& TemporalSpatialRelationalMeasurement::child_frame() const {
    return child_frame_;
}

inline std::string& TemporalSpatialRelationalMeasurement::child_frame() {
    return const_cast<std::string&>(std::as_const(*this).child_frame());
}

}

#endif
