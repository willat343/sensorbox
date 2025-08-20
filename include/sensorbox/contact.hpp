#ifndef SENSORBOX_CONTACT_HPP
#define SENSORBOX_CONTACT_HPP

#include <unordered_map>

#include "sensorbox/measurement.hpp"
#include "sensorbox/sensor.hpp"

namespace sensorbox {

class ContactClassifications : public TemporalMeasurement {
public:
    using typename TemporalMeasurement::Clock;
    using typename TemporalMeasurement::Duration;
    using typename TemporalMeasurement::Timestamp;

    explicit ContactClassifications();

    /**
     * @brief Construct a contact classifications measurement, representing the boolean (binary) contact states of
     * links.
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

#include "sensorbox/impl/contact.hpp"

#endif
