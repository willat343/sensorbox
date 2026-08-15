#ifndef SENSORBOX_STIFFNESS_HPP
#define SENSORBOX_STIFFNESS_HPP

#include <Eigen/Core>
#include <mathbox/covariance.hpp>
#include <nlohmann/json.hpp>

namespace sensorbox {

// TODO: Consider replacing covariance_from_config and stiffness_from_config with uncertainty_from_config, where
// uncertainty holds the stddev, variance, covariance, stiffness, information, etc. quantities

/**
 * @brief Get a covariance matrix from json config containing one of the fields described in `stiffness_from_config`.
 *
 * @tparam Rows size of covariance matrix as `Eigen::Dynamic` or >= 0
 * @param config configuration containing information about uncertainty
 * @return Eigen::Matrix<double, Rows, Rows> covariance matrix
 */
template<int Rows = Eigen::Dynamic>
Eigen::Matrix<double, Rows, Rows> covariance_from_config(const nlohmann::json& config);

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
 * @return Eigen::Matrix<double, Rows, Rows> stiffness matrix
 */
template<int Rows = Eigen::Dynamic>
Eigen::Matrix<double, Rows, Rows> stiffness_from_config(const nlohmann::json& config);

/**
 * @brief Get a covariance density matrix from json config containing one of the following:
 *  - "covariance_density": covariance density matrix as square vector of vectors
 *  - "information_density": information/precision density matrix as square vector of vectors
 *  - "variance_densities": diagonal of covariance density matrix as vector
 *  - "variance_density" and "size": element of diagonal of covariance density matrix as scalar, and vector size
 *  - "sigma_densities": square root of diagonal of covariance density matrix as vector
 *  - "sigma_density" and "size": square root of element of diagonal as scalar, and vector size
 *
 * A covariance density describes uncertainty per unit of some independent variable, so it must be multiplied by an
 * interval of that variable to obtain a covariance. The quantity it describes is defined by its user.
 *
 * @tparam Rows size of covariance density matrix as `Eigen::Dynamic` or >= 0
 * @param config configuration containing information about uncertainty density
 * @return math::CovarianceDensity<double, Rows> covariance density matrix
 */
template<int Rows = Eigen::Dynamic>
math::CovarianceDensity<double, Rows> covariance_density_from_config(const nlohmann::json& config);

/**
 * @brief Get a dynamically-sized covariance density matrix of known size from json config containing one of the fields
 * described in `covariance_density_from_config`. Any vector field must match `size`.
 *
 * @param config configuration containing information about uncertainty density
 * @param size size of covariance density matrix
 * @return math::CovarianceDensityXd covariance density matrix
 */
math::CovarianceDensityXd covariance_density_from_config(const nlohmann::json& config, const int size);

}

#include "sensorbox/impl/stiffness.hpp"

#endif
