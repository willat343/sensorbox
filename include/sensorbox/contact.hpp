#ifndef SENSORBOX_CONTACT_HPP
#define SENSORBOX_CONTACT_HPP

#include <unordered_map>

#include "sensorbox/sensor.hpp"
#include "sensorbox/unary.hpp"

namespace sensorbox {

class ContactClassifications : public UnaryMeasurement {
public:
    using Clock = UnaryMeasurement::Clock;
    using Duration = UnaryMeasurement::Duration;
    using Timestamp = UnaryMeasurement::Timestamp;

    explicit ContactClassifications();

    /**
     * @brief Construct a contact classifications measurement, representing the boolean (binary) contact states of
     * links.
     *
     * The frame of UnaryMeasurement is left empty.
     *
     * @param timestamp_ timestamp
     */
    explicit ContactClassifications(const Timestamp& timestamp_);

    explicit ContactClassifications(const Timestamp& timestamp_,
            const std::unordered_map<std::string, bool>& classifications_);

    const std::unordered_map<std::string, bool>& classifications() const;

    std::unordered_map<std::string, bool>& classifications();

    bool has_classication(const std::string& link) const;

    void set_classication(const std::string& link, const bool classification_);

private:
    /**
     * @brief Map of link names to contact classification state where true is in contact and false is not.
     *
     */
    std::unordered_map<std::string, bool> classifications_;
};

}

#endif
