#include "sensorbox/wrench.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

TEST(wrench_type, dof) {
    EXPECT_EQ(WrenchType(WrenchType::FORCE).dof<2>(), 2);
    EXPECT_EQ(WrenchType(WrenchType::TORQUE).dof<2>(), 1);
    EXPECT_EQ(WrenchType(WrenchType::WRENCH).dof<2>(), 3);
    EXPECT_EQ(WrenchType(WrenchType::FORCE).dof<3>(), 3);
    EXPECT_EQ(WrenchType(WrenchType::TORQUE).dof<3>(), 3);
    EXPECT_EQ(WrenchType(WrenchType::WRENCH).dof<3>(), 6);
}

TEST(wrench_type, offset) {
    EXPECT_EQ(WrenchType(WrenchType::FORCE).offset<2>(), 0);
    EXPECT_EQ(WrenchType(WrenchType::TORQUE).offset<2>(), 2);
    EXPECT_EQ(WrenchType(WrenchType::WRENCH).offset<2>(), 0);
    EXPECT_EQ(WrenchType(WrenchType::FORCE).offset<3>(), 0);
    EXPECT_EQ(WrenchType(WrenchType::TORQUE).offset<3>(), 3);
    EXPECT_EQ(WrenchType(WrenchType::WRENCH).offset<3>(), 0);
}

TEST(wrench_type, components_within_wrench) {
    for (const WrenchTypeBase::Identifiers identifier : WrenchTypeBase::identifiers) {
        const WrenchType type{identifier};
        EXPECT_GE(type.offset<2>(), 0) << std::string(type);
        EXPECT_LE(type.offset<2>() + type.dof<2>(), WrenchMeasurement<2>::ForceDoF + WrenchMeasurement<2>::TorqueDoF)
                << std::string(type);
        EXPECT_GE(type.offset<3>(), 0) << std::string(type);
        EXPECT_LE(type.offset<3>() + type.dof<3>(), WrenchMeasurement<3>::ForceDoF + WrenchMeasurement<3>::TorqueDoF)
                << std::string(type);
    }
}

TEST(wrench_type, string_conversion) {
    EXPECT_EQ(WrenchType(std::string("TORQUE")), WrenchType::TORQUE);
    EXPECT_EQ(std::string(WrenchType(WrenchType::TORQUE)), std::string("TORQUE"));
    EXPECT_THROW(WrenchType(std::string("NOT_A_WRENCH_TYPE")), std::invalid_argument);
}

TEST(wrench_measurement, dimensions) {
    EXPECT_EQ(WrenchMeasurement<2>::D, 2);
    EXPECT_EQ(WrenchMeasurement<2>::ForceDoF, 2);
    EXPECT_EQ(WrenchMeasurement<2>::TorqueDoF, 1);
    EXPECT_EQ(WrenchMeasurement<3>::D, 3);
    EXPECT_EQ(WrenchMeasurement<3>::ForceDoF, 3);
    EXPECT_EQ(WrenchMeasurement<3>::TorqueDoF, 3);
    EXPECT_EQ(WrenchMeasurements<2>::D, 2);
    EXPECT_EQ(WrenchMeasurements<3>::D, 3);
}

TEST(wrench_measurement, default_construction) {
    const WrenchMeasurement<3> measurement;
    EXPECT_EQ(measurement.timestamp(), WrenchMeasurement<3>::Timestamp{WrenchMeasurement<3>::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
    EXPECT_FALSE(measurement.force().has_value());
    EXPECT_FALSE(measurement.torque().has_value());
    EXPECT_THROW(measurement.type(), std::runtime_error);
}

TEST(wrench_measurement, construction_with_force) {
    const WrenchMeasurement<3>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const WrenchMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(0), std::nullopt};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    ASSERT_TRUE(measurement.force().has_value());
    EXPECT_TRUE(measurement.force()->isApprox(test_vector3(0)));
    EXPECT_FALSE(measurement.torque().has_value());
    EXPECT_EQ(measurement.type(), WrenchType::FORCE);
}

TEST(wrench_measurement, construction_with_torque) {
    const WrenchMeasurement<3>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const WrenchMeasurement<3> measurement{timestamp, test_string(0), test_string(0), std::nullopt, test_vector3(1)};
    EXPECT_EQ(measurement.frame(), measurement.child_frame());
    EXPECT_FALSE(measurement.force().has_value());
    ASSERT_TRUE(measurement.torque().has_value());
    EXPECT_TRUE(measurement.torque()->isApprox(test_vector3(1)));
    EXPECT_EQ(measurement.type(), WrenchType::TORQUE);
}

TEST(wrench_measurement, construction_with_wrench) {
    const WrenchMeasurement<3>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const WrenchMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_vector3(0), test_vector3(1)};
    ASSERT_TRUE(measurement.force().has_value());
    EXPECT_TRUE(measurement.force()->isApprox(test_vector3(0)));
    ASSERT_TRUE(measurement.torque().has_value());
    EXPECT_TRUE(measurement.torque()->isApprox(test_vector3(1)));
    EXPECT_EQ(measurement.type(), WrenchType::WRENCH);
}

TEST(wrench_measurement, construction_2d) {
    const WrenchMeasurement<2>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const WrenchMeasurement<2>::Force force{1.0, 2.0};
    const WrenchMeasurement<2>::Torque torque{WrenchMeasurement<2>::Torque::Constant(3.0)};
    const WrenchMeasurement<2> measurement{timestamp, test_string(0), test_string(1), force, torque};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    ASSERT_TRUE(measurement.force().has_value());
    EXPECT_TRUE(measurement.force()->isApprox(force));
    ASSERT_TRUE(measurement.torque().has_value());
    EXPECT_EQ((*measurement.torque())[0], 3.0);
    EXPECT_EQ(measurement.type(), WrenchType::WRENCH);
}

TEST(wrench_measurement, mutable_access) {
    WrenchMeasurement<3> measurement;
    measurement.force() = test_vector3(2);
    EXPECT_EQ(measurement.type(), WrenchType::FORCE);
    measurement.torque() = test_vector3(3);
    EXPECT_EQ(measurement.type(), WrenchType::WRENCH);
    measurement.force().reset();
    EXPECT_EQ(measurement.type(), WrenchType::TORQUE);
    EXPECT_TRUE(measurement.torque()->isApprox(test_vector3(3)));
    measurement.torque().reset();
    EXPECT_THROW(measurement.type(), std::runtime_error);
}

TEST(wrench_measurements, default_construction) {
    const WrenchMeasurements<3> measurements;
    EXPECT_EQ(measurements.timestamp(), WrenchMeasurements<3>::Timestamp{WrenchMeasurements<3>::Duration::zero()});
    EXPECT_EQ(measurements.size(), 0);
    EXPECT_TRUE(measurements.measurements().empty());
}

TEST(wrench_measurements, construction_with_timestamp) {
    const WrenchMeasurements<3>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const WrenchMeasurements<3> measurements{timestamp};
    EXPECT_EQ(measurements.timestamp(), timestamp);
    EXPECT_EQ(measurements.size(), 0);
}

TEST(wrench_measurements, construction_with_measurements) {
    const WrenchMeasurements<3>::Timestamp timestamp{std::chrono::nanoseconds{42}};
    const std::vector<WrenchMeasurement<3>> initial{
            WrenchMeasurement<3>{timestamp, test_string(0), test_string(1), test_vector3(0), std::nullopt},
            WrenchMeasurement<3>{timestamp, test_string(0), test_string(2), test_vector3(1), test_vector3(2)}};
    const WrenchMeasurements<3> measurements{timestamp, initial};
    EXPECT_EQ(measurements.timestamp(), timestamp);
    ASSERT_EQ(measurements.size(), 2);
    EXPECT_EQ(measurements.measurements()[0].child_frame(), test_string(1));
    EXPECT_EQ(measurements.measurements()[0].type(), WrenchType::FORCE);
    EXPECT_EQ(measurements.measurements()[1].child_frame(), test_string(2));
    EXPECT_EQ(measurements.measurements()[1].type(), WrenchType::WRENCH);
    EXPECT_TRUE(measurements.measurements()[1].torque()->isApprox(test_vector3(2)));
}

TEST(wrench_measurements, mutable_access) {
    WrenchMeasurements<3> measurements;
    measurements.measurements().emplace_back(WrenchMeasurement<3>::Timestamp{std::chrono::nanoseconds{42}},
            test_string(0), test_string(1), std::nullopt, test_vector3(0));
    ASSERT_EQ(measurements.size(), 1);
    EXPECT_EQ(measurements.measurements()[0].type(), WrenchType::TORQUE);
    measurements.measurements().clear();
    EXPECT_EQ(measurements.size(), 0);
}

TEST(wrench_measurements, concepts) {
    static_assert(IsTemporalSpatialRelationalMeasurement<WrenchMeasurement<2>>);
    static_assert(IsTemporalSpatialRelationalMeasurement<WrenchMeasurement<3>>);
    static_assert(IsTemporalMeasurement<WrenchMeasurements<2>>);
    static_assert(IsTemporalMeasurement<WrenchMeasurements<3>>);
    static_assert(!IsTemporalSpatialMeasurement<WrenchMeasurements<3>>);
}

}
