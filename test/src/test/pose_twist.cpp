#include "sensorbox/pose_twist.hpp"

#include <gtest/gtest.h>

#include <chrono>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Pose = PoseTwistMeasurement<3>::Pose;
using Timestamp = PoseTwistMeasurement<3>::Timestamp;
using Twist = PoseTwistMeasurement<3>::Twist;

TEST(pose_twist_measurement, default_construction) {
    const PoseTwistMeasurement<3> measurement;
    EXPECT_EQ(measurement.timestamp(), Timestamp{PoseTwistMeasurement<3>::Duration::zero()});
    EXPECT_TRUE(measurement.frame().empty());
    EXPECT_TRUE(measurement.child_frame().empty());
    EXPECT_TRUE(measurement.pose().isApprox(Pose::Identity()));
    EXPECT_TRUE(measurement.twist().isZero());
}

TEST(pose_twist_measurement, dimensions) {
    EXPECT_EQ(PoseTwistMeasurement<2>::D, 2);
    EXPECT_EQ(PoseTwistMeasurement<2>::PoseDoF, 3);
    EXPECT_EQ(PoseTwistMeasurement<3>::D, 3);
    EXPECT_EQ(PoseTwistMeasurement<3>::PoseDoF, 6);
}

TEST(pose_twist_measurement, construction) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const Twist twist{Twist::LinSpaced(0.1, 0.6)};
    const PoseTwistMeasurement<3> measurement{timestamp, test_string(0), test_string(1), test_isometry3(1), twist};
    EXPECT_EQ(measurement.timestamp(), timestamp);
    EXPECT_EQ(measurement.frame(), test_string(0));
    EXPECT_EQ(measurement.child_frame(), test_string(1));
    EXPECT_TRUE(measurement.pose().isApprox(test_isometry3(1)));
    EXPECT_TRUE(measurement.twist().isApprox(twist));
}

TEST(pose_twist_measurement, mutable_access) {
    PoseTwistMeasurement<3> measurement;
    measurement.pose() = test_isometry3(2);
    measurement.twist() = Twist::Ones();
    EXPECT_TRUE(measurement.pose().isApprox(test_isometry3(2)));
    EXPECT_TRUE(measurement.twist().isApprox(Twist::Ones()));
}

}
