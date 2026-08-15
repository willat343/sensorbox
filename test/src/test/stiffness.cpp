#include "sensorbox/stiffness.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace sensorbox {

TEST(stiffness_from_config, sigma) {
    const nlohmann::json config = {{"sigma", 0.5}};
    const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() * 2.0;
    EXPECT_TRUE(stiffness_from_config<3>(config).isApprox(expected));
}

TEST(stiffness_from_config, sigmas) {
    const nlohmann::json config = {{"sigmas", {0.5, 0.25, 0.125}}};
    const Eigen::Matrix3d expected = Eigen::Vector3d{2.0, 4.0, 8.0}.asDiagonal();
    EXPECT_TRUE(stiffness_from_config<3>(config).isApprox(expected));
}

TEST(stiffness_from_config, variance) {
    const nlohmann::json config = {{"variance", 0.25}};
    const Eigen::Matrix2d expected = Eigen::Matrix2d::Identity() * 2.0;
    EXPECT_TRUE(stiffness_from_config<2>(config).isApprox(expected));
}

TEST(stiffness_from_config, variances) {
    const nlohmann::json config = {{"variances", {0.25, 0.0625}}};
    const Eigen::Matrix2d expected = Eigen::Vector2d{2.0, 4.0}.asDiagonal();
    EXPECT_TRUE(stiffness_from_config<2>(config).isApprox(expected));
}

TEST(stiffness_from_config, dynamic_sigmas) {
    const nlohmann::json config = {{"sigmas", {0.5, 0.25, 0.125, 0.0625, 0.5, 0.25}}};
    const Eigen::MatrixXd stiffness = stiffness_from_config<Eigen::Dynamic>(config);
    ASSERT_EQ(stiffness.rows(), 6);
    ASSERT_EQ(stiffness.cols(), 6);
    EXPECT_TRUE(stiffness.isApprox(stiffness_from_config<6>(config)));
}

TEST(stiffness_from_config, dynamic_sigma_requires_size) {
    EXPECT_THROW(stiffness_from_config<Eigen::Dynamic>(nlohmann::json{{"sigma", 0.5}}), std::runtime_error);
    const nlohmann::json config = {{"sigma", 0.5}, {"size", 4}};
    const Eigen::MatrixXd stiffness = stiffness_from_config<Eigen::Dynamic>(config);
    ASSERT_EQ(stiffness.rows(), 4);
    EXPECT_TRUE(stiffness.isApprox(Eigen::MatrixXd::Identity(4, 4) * 2.0));
}

TEST(stiffness_from_config, dynamic_variance_requires_size) {
    EXPECT_THROW(stiffness_from_config<Eigen::Dynamic>(nlohmann::json{{"variance", 0.25}}), std::runtime_error);
    const nlohmann::json config = {{"variance", 0.25}, {"size", 2}};
    EXPECT_TRUE(stiffness_from_config<Eigen::Dynamic>(config).isApprox(Eigen::MatrixXd::Identity(2, 2) * 2.0));
}

TEST(stiffness_from_config, wrong_size_throws) {
    EXPECT_THROW(stiffness_from_config<3>(nlohmann::json{{"sigmas", {0.5, 0.25}}}), std::runtime_error);
    EXPECT_THROW(stiffness_from_config<3>(nlohmann::json{{"variances", {0.5, 0.25}}}), std::runtime_error);
}

TEST(stiffness_from_config, missing_field_throws) {
    EXPECT_THROW(stiffness_from_config<3>(nlohmann::json::object()), std::runtime_error);
}

TEST(stiffness_from_config, unimplemented_fields_throw) {
    EXPECT_THROW(stiffness_from_config<1>(nlohmann::json{{"covariance", {{1.0}}}}), std::runtime_error);
    EXPECT_THROW(stiffness_from_config<1>(nlohmann::json{{"information", {{1.0}}}}), std::runtime_error);
}

TEST(covariance_from_config, sigma) {
    const nlohmann::json config = {{"sigma", 0.5}};
    const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() * 0.25;
    EXPECT_TRUE(covariance_from_config<3>(config).isApprox(expected));
}

TEST(covariance_from_config, sigmas) {
    const nlohmann::json config = {{"sigmas", {0.5, 0.25}}};
    const Eigen::Matrix2d expected = Eigen::Vector2d{0.25, 0.0625}.asDiagonal();
    EXPECT_TRUE(covariance_from_config<2>(config).isApprox(expected));
}

TEST(covariance_from_config, variance) {
    const nlohmann::json config = {{"variance", 0.25}};
    EXPECT_TRUE(covariance_from_config<2>(config).isApprox(Eigen::Matrix2d::Identity() * 0.25));
}

TEST(covariance_from_config, variances) {
    const nlohmann::json config = {{"variances", {0.25, 0.5, 0.75}}};
    const Eigen::Matrix3d expected = Eigen::Vector3d{0.25, 0.5, 0.75}.asDiagonal();
    EXPECT_TRUE(covariance_from_config<3>(config).isApprox(expected));
}

TEST(covariance_from_config, dynamic) {
    const nlohmann::json config = {{"variances", {0.25, 0.5, 0.75}}};
    const Eigen::MatrixXd covariance = covariance_from_config<Eigen::Dynamic>(config);
    ASSERT_EQ(covariance.rows(), 3);
    EXPECT_TRUE(covariance.isApprox(covariance_from_config<3>(config)));
}

TEST(covariance_from_config, is_inverse_of_squared_stiffness) {
    const nlohmann::json config = {{"sigmas", {0.5, 0.25, 0.125}}};
    const Eigen::Matrix3d covariance = covariance_from_config<3>(config);
    const Eigen::Matrix3d stiffness = stiffness_from_config<3>(config);
    EXPECT_TRUE((stiffness.transpose() * stiffness).isApprox(covariance.inverse()));
}

TEST(covariance_from_config, missing_field_throws) {
    EXPECT_THROW(covariance_from_config<3>(nlohmann::json::object()), std::runtime_error);
    EXPECT_THROW(covariance_from_config<3>(nlohmann::json{{"sigmas", {0.5, 0.25}}}), std::runtime_error);
}

}
