#include "sensorbox/measurement.hpp"

#include <gtest/gtest.h>

#include <chrono>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Duration = TemporalMeasurement::Duration;
using Timestamp = TemporalMeasurement::Timestamp;

TEST(temporal_measurement, default_construction) {
    const TemporalMeasurement measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{Duration::zero()});
}

TEST(temporal_measurement, construction) {
    const Timestamp timestamp{std::chrono::seconds{12} + std::chrono::nanoseconds{345}};
    const TemporalMeasurement measurement{timestamp};
    EXPECT_EQ(measurement.timestamp(), timestamp);
}

TEST(temporal_measurement, mutable_access) {
    const Timestamp timestamp{std::chrono::nanoseconds{999}};
    TemporalMeasurement measurement;
    measurement.timestamp() = timestamp;
    EXPECT_EQ(measurement.timestamp(), timestamp);
}

TEST(temporal_spatial_measurement, default_construction) {
    const TemporalSpatialMeasurement measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
}

TEST(temporal_spatial_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{7}};
    const TemporalSpatialMeasurement measurement{timestamp, test_string(0)};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
}

TEST(temporal_spatial_measurement, mutable_access) {
    TemporalSpatialMeasurement measurement;
    measurement.frame() = test_string(1);
    EXPECT_EQ(measurement.frame(), test_string(1));
}

TEST(temporal_spatial_relational_measurement, default_construction) {
    const TemporalSpatialRelationalMeasurement measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
}

TEST(temporal_spatial_relational_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{8}};
    const TemporalSpatialRelationalMeasurement measurement{timestamp, test_string(0), test_string(1)};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
}

TEST(temporal_spatial_relational_measurement, mutable_access) {
    TemporalSpatialRelationalMeasurement measurement;
    measurement.child_frame() = test_string(2);
    EXPECT_EQ(measurement.child_frame(), test_string(2));
}

TEST(measurement_concepts, hierarchy) {
    static_assert(IsTemporalMeasurement<TemporalMeasurement>);
    static_assert(IsTemporalMeasurement<TemporalSpatialMeasurement>);
    static_assert(IsTemporalMeasurement<TemporalSpatialRelationalMeasurement>);
    static_assert(!IsTemporalSpatialMeasurement<TemporalMeasurement>);
    static_assert(IsTemporalSpatialMeasurement<TemporalSpatialMeasurement>);
    static_assert(IsTemporalSpatialMeasurement<TemporalSpatialRelationalMeasurement>);
    static_assert(!IsTemporalSpatialRelationalMeasurement<TemporalSpatialMeasurement>);
    static_assert(IsTemporalSpatialRelationalMeasurement<TemporalSpatialRelationalMeasurement>);
    static_assert(!IsTemporalMeasurement<double>);
}

TEST(measurement_type, string_conversion) {
    for (const MeasurementType measurement_type : MeasurementType::values()) {
        EXPECT_EQ(MeasurementType(std::string(measurement_type)), measurement_type);
    }
}

}
