#include "sensorbox/unary.hpp"

#include <utility>

namespace sensorbox {

UnaryMeasurement::UnaryMeasurement() : UnaryMeasurement(Timestamp(Duration::zero()), std::string(), std::string()) {}

UnaryMeasurement::UnaryMeasurement(const Timestamp& timestamp_, const std::string& frame_)
    : UnaryMeasurement(timestamp_, frame_, frame_) {}

UnaryMeasurement::UnaryMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_)
    : timestamp_(timestamp_), frame_(frame_), child_frame_(child_frame_) {}

const std::string& UnaryMeasurement::child_frame() const {
    return child_frame_;
}

std::string& UnaryMeasurement::child_frame() {
    return const_cast<std::string&>(std::as_const(*this).child_frame());
}

const std::string& UnaryMeasurement::frame() const {
    return frame_;
}

std::string& UnaryMeasurement::frame() {
    return const_cast<std::string&>(std::as_const(*this).frame());
}

auto UnaryMeasurement::timestamp() const -> const Timestamp& {
    return timestamp_;
}

auto UnaryMeasurement::timestamp() -> Timestamp& {
    return const_cast<Timestamp&>(std::as_const(*this).timestamp());
}

}
