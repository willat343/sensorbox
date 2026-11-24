#include <gtest/gtest.h>

#include <chrono>
#include <cppbox/time.hpp>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "sensorbox/json/convert.hpp"
#include "sensorbox/json_loadable.hpp"
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
    EXPECT_THROW(j.get<SensorType>(), std::invalid_argument);
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

TEST(json_convert, to_json_wrench_measurements_3d) {
    // A force and a full wrench, in the order of the measurements, valid against the 3D schema
    const WrenchMeasurements<3>::Timestamp timestamp{std::chrono::nanoseconds{123456789}};
    const WrenchMeasurements<3> wrench_measurements{timestamp,
            {WrenchMeasurement<3>{timestamp, "world", "foot", WrenchMeasurement<3>::Force{1.0, 2.0, 3.0}, std::nullopt},
                    WrenchMeasurement<3>{timestamp, "base", "base", WrenchMeasurement<3>::Force{4.0, 5.0, 6.0},
                            WrenchMeasurement<3>::Torque{7.0, 8.0, 9.0}}}};
    const nlohmann::json j = wrench_measurements;
    EXPECT_EQ(j["timestamp"].get<std::int64_t>(), cppbox::to_nsec(timestamp));
    ASSERT_EQ(j["wrenches"].size(), 2);
    const nlohmann::json& force = j["wrenches"][0];
    EXPECT_EQ(force["frame_id"].get<std::string>(), "world");
    EXPECT_EQ(force["child_frame_id"].get<std::string>(), "foot");
    EXPECT_EQ(force["force"], (nlohmann::json{{"x", 1.0}, {"y", 2.0}, {"z", 3.0}}));
    EXPECT_FALSE(force.contains("torque"));
    const nlohmann::json& wrench = j["wrenches"][1];
    EXPECT_EQ(wrench["force"], (nlohmann::json{{"x", 4.0}, {"y", 5.0}, {"z", 6.0}}));
    EXPECT_EQ(wrench["torque"], (nlohmann::json{{"x", 7.0}, {"y", 8.0}, {"z", 9.0}}));
    nlohmann::json schema;
    sensorbox_schema_loader(nlohmann::json_uri("TemporalWrenches3D.schema.json"), schema);
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(schema);
    EXPECT_NO_THROW(validator.validate(j));
}

TEST(json_convert, to_json_wrench_measurements_2d) {
    // A torque is a scalar in 2D, valid against the 2D schema but not the 3D one
    const WrenchMeasurements<2>::Timestamp timestamp{std::chrono::nanoseconds{5}};
    const WrenchMeasurements<2> wrench_measurements{timestamp,
            {WrenchMeasurement<2>{timestamp, "world", "foot", WrenchMeasurement<2>::Force{1.0, 2.0},
                    WrenchMeasurement<2>::Torque{3.0}}}};
    const nlohmann::json j = wrench_measurements;
    ASSERT_EQ(j["wrenches"].size(), 1);
    EXPECT_EQ(j["wrenches"][0]["force"], (nlohmann::json{{"x", 1.0}, {"y", 2.0}}));
    EXPECT_EQ(j["wrenches"][0]["torque"].get<double>(), 3.0);
    for (const int D : {2, 3}) {
        nlohmann::json schema;
        sensorbox_schema_loader(nlohmann::json_uri("TemporalWrenches" + std::to_string(D) + "D.schema.json"), schema);
        nlohmann::json_schema::json_validator validator;
        validator.set_root_schema(schema);
        if (D == 2) {
            EXPECT_NO_THROW(validator.validate(j));
        } else {
            EXPECT_THROW(validator.validate(j), std::invalid_argument);
        }
    }
}

TEST(json_convert, to_json_empty_wrench_measurements) {
    const nlohmann::json j = WrenchMeasurements<3>{};
    EXPECT_EQ(j["timestamp"].get<std::int64_t>(), 0);
    EXPECT_TRUE(j["wrenches"].is_array());
    EXPECT_TRUE(j["wrenches"].empty());
}

}
