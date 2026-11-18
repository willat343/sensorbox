#ifndef SENSORBOX_IMPL_WRENCH_HPP
#define SENSORBOX_IMPL_WRENCH_HPP

#include <cppbox/exceptions.hpp>
#include <utility>

#include "sensorbox/wrench.hpp"

namespace sensorbox {

inline WrenchType::WrenchType() : WrenchTypeBase() {}

inline WrenchType::WrenchType(const WrenchTypeBase::Identifiers identifier) : WrenchTypeBase(identifier) {}

inline WrenchType::WrenchType(const std::string& string) : WrenchTypeBase(string) {}

template<int D>
inline int WrenchType::dof() const {
    switch ((*this)()) {
        case FORCE:
            return WrenchMeasurement<D>::ForceDoF;
        case TORQUE:
            return WrenchMeasurement<D>::TorqueDoF;
        case WRENCH:
            return WrenchMeasurement<D>::ForceDoF + WrenchMeasurement<D>::TorqueDoF;
        default:
            throw_here("Unhandled WrenchType " + std::string(*this) + ".");
    }
}

template<int D>
inline int WrenchType::offset() const {
    switch ((*this)()) {
        case FORCE:
            return 0;
        case TORQUE:
            return WrenchMeasurement<D>::ForceDoF;
        case WRENCH:
            return 0;
        default:
            throw_here("Unhandled WrenchType " + std::string(*this) + ".");
    }
}

template<int D_>
inline WrenchMeasurement<D_>::WrenchMeasurement()
    : WrenchMeasurement(Timestamp{Duration::zero()}, std::string(), std::string(), std::nullopt, std::nullopt) {}

template<int D_>
inline WrenchMeasurement<D_>::WrenchMeasurement(const Timestamp& timestamp_, const std::string& frame_,
        const std::string& child_frame_, const std::optional<Force>& force_, const std::optional<Torque>& torque_)
    : TemporalSpatialRelationalMeasurement(timestamp_, frame_, child_frame_), force_(force_), torque_(torque_) {}

template<int D_>
inline auto WrenchMeasurement<D_>::force() const -> const std::optional<Force>& {
    return force_;
}

template<int D_>
inline auto WrenchMeasurement<D_>::force() -> std::optional<Force>& {
    return const_cast<std::optional<Force>&>(std::as_const(*this).force());
}

template<int D_>
inline auto WrenchMeasurement<D_>::torque() const -> const std::optional<Torque>& {
    return torque_;
}

template<int D_>
inline auto WrenchMeasurement<D_>::torque() -> std::optional<Torque>& {
    return const_cast<std::optional<Torque>&>(std::as_const(*this).torque());
}

template<int D_>
inline WrenchType WrenchMeasurement<D_>::type() const {
    throw_if(!force() && !torque(), "Wrench measurement has neither a force nor a torque, so it has no type.");
    return force() ? (torque() ? WrenchType::WRENCH : WrenchType::FORCE) : WrenchType::TORQUE;
}

template<int D_>
inline WrenchMeasurements<D_>::WrenchMeasurements() : WrenchMeasurements(Timestamp{Duration::zero()}) {}

template<int D_>
inline WrenchMeasurements<D_>::WrenchMeasurements(const Timestamp& timestamp_) : WrenchMeasurements(timestamp_, {}) {}

template<int D_>
inline WrenchMeasurements<D_>::WrenchMeasurements(const Timestamp& timestamp_,
        const std::vector<WrenchMeasurement<D>>& measurements_)
    : TemporalMeasurement(timestamp_), measurements_(measurements_) {}

template<int D_>
inline auto WrenchMeasurements<D_>::measurements() const -> const std::vector<WrenchMeasurement<D>>& {
    return measurements_;
}

template<int D_>
inline auto WrenchMeasurements<D_>::measurements() -> std::vector<WrenchMeasurement<D>>& {
    return const_cast<std::vector<WrenchMeasurement<D>>&>(std::as_const(*this).measurements());
}

template<int D_>
inline std::size_t WrenchMeasurements<D_>::size() const {
    return measurements().size();
}

}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template class WrenchMeasurement<2>;
extern template class WrenchMeasurement<3>;
extern template class WrenchMeasurements<2>;
extern template class WrenchMeasurements<3>;

}
#endif

#endif
