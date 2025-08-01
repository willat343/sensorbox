#ifndef SENSORBOX_IMPL_STIFFNESS_HPP
#define SENSORBOX_IMPL_STIFFNESS_HPP

#include <convert/convert.hpp>
#include <mathbox/stiffness.hpp>
#include <stdexcept>

#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template<int Rows>
Eigen::Matrix<double, Rows, Rows> stiffness_from_config(const nlohmann::json& config) {
    static_assert(Rows == Eigen::Dynamic || Rows >= 0, "stiffness_from_config: Rows must be Eigen::Dynamic or >= 0.");
    if (config.contains("covariance")) {
        throw std::runtime_error("stiffness_from_config: Conversion of covariance to stiffness not yet implemented.");
    } else if (config.contains("information")) {
        throw std::runtime_error("stiffness_from_config: Conversion of information to stiffness not yet implemented.");
    } else if (config.contains("variances")) {
        Eigen::Matrix<double, Rows, 1> variances;
        if constexpr (Rows == Eigen::Dynamic) {
            variances = convert::to<Eigen::Matrix<double, Rows, 1>>(
                    config["variances"].template get<std::vector<double>>());
        } else {
            const int config_size = config["variances"].size();
            if (config_size != Rows) {
                throw std::runtime_error("stiffness_from_config: Expected size of variances vector in json was " +
                                         std::to_string(Rows) + " but was " + std::to_string(config_size) + ".");
            }
            variances = convert::to<Eigen::Matrix<double, Rows, 1>>(
                    config["variances"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::stiffness_from_variances(variances);
    } else if (config.contains("variance")) {
        if constexpr (Rows == Eigen::Dynamic) {
            if (!config.contains("size")) {
                throw std::runtime_error("stiffness_from_config: Expected size field for Rows == Eigen::Dynamic.");
            }
            return math::stiffness_from_variance(config["variance"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::stiffness_from_variance<Rows>(config["variance"].template get<double>());
        }
    } else if (config.contains("sigmas")) {
        Eigen::Matrix<double, Rows, 1> sigmas;
        if constexpr (Rows == Eigen::Dynamic) {
            sigmas = convert::to<Eigen::Matrix<double, Rows, 1>>(config["sigmas"].template get<std::vector<double>>());
        } else {
            const int config_size = config["sigmas"].size();
            if (config_size != Rows) {
                throw std::runtime_error("stiffness_from_config: Expected size of sigmas vector in json was " +
                                         std::to_string(Rows) + " but was " + std::to_string(config_size) + ".");
            }
            sigmas = convert::to<Eigen::Matrix<double, Rows, 1>>(
                    config["sigmas"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::stiffness_from_sigmas(sigmas);
    } else if (config.contains("sigma")) {
        if constexpr (Rows == Eigen::Dynamic) {
            if (!config.contains("size")) {
                throw std::runtime_error("stiffness_from_config: Expected size field for Rows == Eigen::Dynamic.");
            }
            return math::stiffness_from_sigma(config["sigma"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::stiffness_from_sigma<Rows>(config["sigma"].template get<double>());
        }
    } else {
        throw std::runtime_error("stiffness_from_config: Field covariance, information, variances, variance, sigmas or "
                                 "sigma missing from config. Config was:\n" +
                                 config.dump());
    }
}

}

#endif
