#ifndef SENSORBOX_TEST_TEST_INSTANCES_HPP
#define SENSORBOX_TEST_TEST_INSTANCES_HPP

#include "sensorbox/imu.hpp"
#include "sensorbox/test/test_instances.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sensorbox {

Eigen::Matrix3d test_covariance_3x3(const unsigned int i);

template<typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> test_dynamic_covariance_3x3(const unsigned int i) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cov(3, 3);
    switch (i) {
        case 0:
            cov <<  0.1, 0.0, 0.0,
                    0.0, 0.1, 0.0,
                    0.0, 0.0, 0.1;
            break;
        case 1:
            cov <<  0.1, 0.0, 0.0,
                    0.0, 0.2, 0.0,
                    0.0, 0.0, 0.3;
            break;
        case 2:
            cov <<  0.10, 0.01, 0.02,
                    0.01, 0.20, 0.07,
                    0.02, 0.07, 0.30;
            break;
        default:
            cov <<  0.1, 0.002, 0.003, 
                    0.002, 0.2, 0.006,
                    0.003, 0.006, 0.3;
            cov *= static_cast<Scalar>(i);
            break;
    }
    return cov;
}

Eigen::Matrix<double, 6, 6> test_covariance_6x6(const unsigned int i);

Imu test_imu(const unsigned int i);

Eigen::Isometry3d test_isometry3(const unsigned int i);

Eigen::Quaterniond test_quaternion(const unsigned int i);

std::string test_string(const unsigned int i);

Imu::TimeStamp test_time(const unsigned int i);

Eigen::Vector3d test_vector3(const unsigned int i);

void pretest_check_covariance_6x6(const unsigned int i);

void pretest_check_quaternion(const unsigned int i);

void pretest_check_vector3(const unsigned int i);

}

#endif
