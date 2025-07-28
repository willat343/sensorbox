#ifndef SENSORBOX_STIFFNESS_HPP
#define SENSORBOX_STIFFNESS_HPP

#include <Eigen/Core>
#include <nlohmann/json.hpp>

namespace sensorbox {

/**
 * @brief Get a stiffness matrix from json config containing one of the following:
 *  - "covariance": covariance matrix as square vector of vectors
 *  - "information": information/precision matrix as square vector of vectors
 *  - "variances": diagonal of covariance matrix as vector
 *  - "variance" and "size": element of diagonal of covariance matrix as scalar, and vector size
 *  - "sigmas": square root of diagonal of covariance matrix as vector
 *  - "sigma" and "size": square root of element of diagonal of covariance matrix as scalar, and vector size
 *
 * @tparam Rows size of stiffness matrix as `Eigen::Dynamic` or >= 0
 * @param config configuration containing information about stiffness
 * @return Eigen::Matrix<double, Rows, 1> stiffness matrix
 */
template<int Rows = Eigen::Dynamic>
Eigen::Matrix<double, Rows, Rows> stiffness_from_config(const nlohmann::json& config);

}

#include "sensorbox/impl/stiffness.hpp"

#endif
