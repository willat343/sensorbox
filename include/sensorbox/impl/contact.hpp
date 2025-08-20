#ifndef SENSORBOX_IMPL_CONTACT_HPP
#define SENSORBOX_IMPL_CONTACT_HPP

#include "sensorbox/contact.hpp"

namespace sensorbox {

inline ContactClassifications::ContactClassifications() : ContactClassifications(Timestamp{Duration::zero()}) {}

inline ContactClassifications::ContactClassifications(const Timestamp& timestamp_)
    : ContactClassifications(timestamp_, {}) {}

inline ContactClassifications::ContactClassifications(const Timestamp& timestamp_,
        const std::unordered_map<std::string, bool>& classifications_)
    : TemporalMeasurement(timestamp_), classifications_(classifications_) {}

inline const std::unordered_map<std::string, bool>& ContactClassifications::classifications() const {
    return classifications_;
}

inline std::unordered_map<std::string, bool>& ContactClassifications::classifications() {
    return const_cast<std::unordered_map<std::string, bool>&>(std::as_const(*this).classifications());
}

inline bool ContactClassifications::has_classication(const std::string& link) const {
    return classifications().contains(link);
}

inline void ContactClassifications::set_classication(const std::string& link, const bool classification_) {
    classifications()[link] = classification_;
}

}

#endif
