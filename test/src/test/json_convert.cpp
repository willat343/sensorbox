#include <gtest/gtest.h>

#include <chrono>
#include <cppbox/time.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "sensorbox/json/convert.hpp"
#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

TEST(json_convert, from_json_sensor_type) {
    for (const SensorTypeBase::Identifiers identifier : SensorTypeBase::identifiers) {
        const SensorType sensor_type{identifier};
        const nlohmann::json j = std::string(sensor_type);
        EXPECT_EQ(j.get<SensorType>(), sensor_type);
    }
}

TEST(json_convert, from_json_invalid_sensor_type_throws) {
    const nlohmann::json j = "NOT_A_SENSOR_TYPE";
    EXPECT_THROW(j.get<SensorType>(), std::runtime_error);
}

TEST(json_convert, to_json_contact_classifications) {
    const ContactClassifications::Timestamp timestamp{std::chrono::nanoseconds{123456789}};
    ContactClassifications classifications{timestamp};
    classifications.set_classification(test_string(0), true);
    classifications.set_classification(test_string(1), false);
    const nlohmann::json j = classifications;
    ASSERT_TRUE(j.contains("timestamp"));
    EXPECT_EQ(j["timestamp"].get<std::int64_t>(), cppbox::to_nsec(timestamp));
    ASSERT_TRUE(j.contains("values"));
    ASSERT_TRUE(j["values"].is_array());
    ASSERT_EQ(j["values"].size(), 2);
    // The order of the values follows the unordered map, so search for each key
    for (const auto& [link, expected_classification] : classifications.classifications()) {
        bool found = false;
        for (const nlohmann::json& value : j["values"]) {
            if (value.at("key").get<std::string>() == link) {
                EXPECT_EQ(value.at("value").get<bool>(), expected_classification);
                found = true;
            }
        }
        EXPECT_TRUE(found) << link;
    }
}

TEST(json_convert, to_json_empty_contact_classifications) {
    const nlohmann::json j = ContactClassifications{};
    EXPECT_EQ(j["timestamp"].get<std::int64_t>(), 0);
    EXPECT_TRUE(j["values"].is_array());
    EXPECT_TRUE(j["values"].empty());
}

}
