#include "sensorbox/contact.hpp"

namespace sensorbox {

ContactClassifications::ContactClassifications() : ContactClassifications(Timestamp(Duration::zero())) {}

ContactClassifications::ContactClassifications(const Timestamp& timestamp_) : ContactClassifications(timestamp_, {}) {}

ContactClassifications::ContactClassifications(const Timestamp& timestamp_,
        const std::unordered_map<std::string, bool>& classifications_)
    : UnaryMeasurement(timestamp_, ""), classifications_(classifications_) {}

const std::unordered_map<std::string, bool>& ContactClassifications::classifications() const {
    return classifications_;
}

std::unordered_map<std::string, bool>& ContactClassifications::classifications() {
    return const_cast<std::unordered_map<std::string, bool>&>(std::as_const(*this).classifications());
}

bool ContactClassifications::has_classication(const std::string& link) const {
    return classifications().contains(link);
}

void ContactClassifications::set_classication(const std::string& link, const bool classification_) {
    classifications()[link] = classification_;
}

}
