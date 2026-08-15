#ifndef SENSORBOX_IMPL_STIFFNESS_HPP
#define SENSORBOX_IMPL_STIFFNESS_HPP

#include <convert/convert.hpp>
#include <cppbox/exceptions.hpp>
#include <mathbox/covariance.hpp>
#include <mathbox/stiffness.hpp>

#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template<int Rows>
Eigen::Matrix<double, Rows, Rows> covariance_from_config(const nlohmann::json& config) {
    static_assert(Rows == Eigen::Dynamic || Rows >= 0, "covariance_from_config: Rows must be Eigen::Dynamic or >= 0.");
    if (config.contains("covariance")) {
        not_implemented("Conversion of covariance to covariance not yet implemented.");
    } else if (config.contains("information")) {
        not_implemented("Conversion of information to covariance not yet implemented.");
    } else if (config.contains("variances")) {
        Eigen::Vector<double, Rows> variances;
        if constexpr (Rows == Eigen::Dynamic) {
            variances =
                    convert::to<Eigen::Vector<double, Rows>>(config["variances"].template get<std::vector<double>>());
        } else {
            const int config_size = config["variances"].size();
            throw_if(config_size != Rows, "covariance_from_config: Expected size of variances vector in json was " +
                                                  std::to_string(Rows) + " but was " + std::to_string(config_size) +
                                                  ".");
            variances = convert::to<Eigen::Vector<double, Rows>>(
                    config["variances"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::covariance_from_variances(variances);
    } else if (config.contains("variance")) {
        if constexpr (Rows == Eigen::Dynamic) {
            throw_if(!config.contains("size"),
                    "covariance_from_config: Expected size field for Rows == Eigen::Dynamic.");
            return math::covariance_from_variance(config["variance"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::covariance_from_variance<Rows>(config["variance"].template get<double>());
        }
    } else if (config.contains("sigmas")) {
        Eigen::Vector<double, Rows> sigmas;
        if constexpr (Rows == Eigen::Dynamic) {
            sigmas = convert::to<Eigen::Vector<double, Rows>>(config["sigmas"].template get<std::vector<double>>());
        } else {
            const int config_size = config["sigmas"].size();
            throw_if(config_size != Rows, "covariance_from_config: Expected size of sigmas vector in json was " +
                                                  std::to_string(Rows) + " but was " + std::to_string(config_size) +
                                                  ".");
            sigmas = convert::to<Eigen::Vector<double, Rows>>(
                    config["sigmas"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::covariance_from_sigmas(sigmas);
    } else if (config.contains("sigma")) {
        if constexpr (Rows == Eigen::Dynamic) {
            throw_if(!config.contains("size"),
                    "covariance_from_config: Expected size field for Rows == Eigen::Dynamic.");
            return math::covariance_from_sigma(config["sigma"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::covariance_from_sigma<Rows>(config["sigma"].template get<double>());
        }
    } else {
        throw_here(
                "covariance_from_config: Field covariance, information, variances, variance, sigmas or sigma missing "
                "from config. Config was:\n" +
                config.dump());
    }
}

template<int Rows>
Eigen::Matrix<double, Rows, Rows> stiffness_from_config(const nlohmann::json& config) {
    static_assert(Rows == Eigen::Dynamic || Rows >= 0, "stiffness_from_config: Rows must be Eigen::Dynamic or >= 0.");
    if (config.contains("covariance")) {
        not_implemented("Conversion of covariance to stiffness not yet implemented.");
    } else if (config.contains("information")) {
        not_implemented("Conversion of information to stiffness not yet implemented.");
    } else if (config.contains("variances")) {
        Eigen::Vector<double, Rows> variances;
        if constexpr (Rows == Eigen::Dynamic) {
            variances =
                    convert::to<Eigen::Vector<double, Rows>>(config["variances"].template get<std::vector<double>>());
        } else {
            const int config_size = config["variances"].size();
            throw_if(config_size != Rows, "stiffness_from_config: Expected size of variances vector in json was " +
                                                  std::to_string(Rows) + " but was " + std::to_string(config_size) +
                                                  ".");
            variances = convert::to<Eigen::Vector<double, Rows>>(
                    config["variances"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::stiffness_from_variances(variances);
    } else if (config.contains("variance")) {
        if constexpr (Rows == Eigen::Dynamic) {
            throw_if(!config.contains("size"),
                    "stiffness_from_config: Expected size field for Rows == Eigen::Dynamic.");
            return math::stiffness_from_variance(config["variance"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::stiffness_from_variance<Rows>(config["variance"].template get<double>());
        }
    } else if (config.contains("sigmas")) {
        Eigen::Vector<double, Rows> sigmas;
        if constexpr (Rows == Eigen::Dynamic) {
            sigmas = convert::to<Eigen::Vector<double, Rows>>(config["sigmas"].template get<std::vector<double>>());
        } else {
            const int config_size = config["sigmas"].size();
            throw_if(config_size != Rows, "stiffness_from_config: Expected size of sigmas vector in json was " +
                                                  std::to_string(Rows) + " but was " + std::to_string(config_size) +
                                                  ".");
            sigmas = convert::to<Eigen::Vector<double, Rows>>(
                    config["sigmas"].template get<std::array<double, std::size_t(Rows)>>());
        }
        return math::stiffness_from_sigmas(sigmas);
    } else if (config.contains("sigma")) {
        if constexpr (Rows == Eigen::Dynamic) {
            throw_if(!config.contains("size"),
                    "stiffness_from_config: Expected size field for Rows == Eigen::Dynamic.");
            return math::stiffness_from_sigma(config["sigma"].template get<double>(),
                    config["size"].template get<int>());
        } else {
            return math::stiffness_from_sigma<Rows>(config["sigma"].template get<double>());
        }
    } else {
        throw_here("stiffness_from_config: Field covariance, information, variances, variance, sigmas or sigma missing "
                   "from config. Config was:\n" +
                   config.dump());
    }
}

template<int Rows>
inline math::CovarianceDensity<double, Rows> covariance_density_from_config(const nlohmann::json& config) {
    static_assert(Rows == Eigen::Dynamic || Rows >= 0, "Rows must be Eigen::Dynamic or >= 0.");
    if constexpr (Rows == Eigen::Dynamic) {
        throw_if(!config.contains("size"), "Expected size field for Rows == Eigen::Dynamic.");
        return covariance_density_from_config(config, config["size"].template get<int>());
    } else {
        return math::CovarianceDensity<double, Rows>{covariance_density_from_config(config, Rows)};
    }
}

inline math::CovarianceDensityXd covariance_density_from_config(const nlohmann::json& config, const int size) {
    if (config.contains("covariance_density")) {
        not_implemented("Conversion of covariance density to covariance density not yet implemented.");
    } else if (config.contains("information_density")) {
        not_implemented("Conversion of information density to covariance density not yet implemented.");
    } else if (config.contains("variance_densities")) {
        const int config_size = config["variance_densities"].size();
        throw_if(config_size != size, "Expected size of variance_densities vector in json was " + std::to_string(size) +
                                              " but was " + std::to_string(config_size) + ".");
        return math::CovarianceDensityXd{math::covariance_from_variances(
                convert::to<Eigen::VectorXd>(config["variance_densities"].template get<std::vector<double>>()))};
    } else if (config.contains("variance_density")) {
        return math::CovarianceDensityXd{
                math::covariance_from_variance(config["variance_density"].template get<double>(), size)};
    } else if (config.contains("sigma_densities")) {
        const int config_size = config["sigma_densities"].size();
        throw_if(config_size != size, "Expected size of sigma_densities vector in json was " + std::to_string(size) +
                                              " but was " + std::to_string(config_size) + ".");
        return math::CovarianceDensityXd{math::covariance_from_sigmas(
                convert::to<Eigen::VectorXd>(config["sigma_densities"].template get<std::vector<double>>()))};
    } else if (config.contains("sigma_density")) {
        return math::CovarianceDensityXd{
                math::covariance_from_sigma(config["sigma_density"].template get<double>(), size)};
    } else {
        throw_here(
                "Field covariance_density, information_density, variance_densities, variance_density, sigma_densities "
                "or sigma_density missing from config. Config was:\n" +
                config.dump());
    }
}
}

#if !SENSORBOX_HEADER_ONLY
namespace sensorbox {

extern template Eigen::MatrixXd stiffness_from_config<Eigen::Dynamic>(const nlohmann::json&);
extern template Eigen::Matrix<double, 1, 1> stiffness_from_config<1>(const nlohmann::json&);
extern template Eigen::Matrix<double, 2, 2> stiffness_from_config<2>(const nlohmann::json&);
extern template Eigen::Matrix<double, 3, 3> stiffness_from_config<3>(const nlohmann::json&);
extern template Eigen::Matrix<double, 6, 6> stiffness_from_config<6>(const nlohmann::json&);

}
#endif

#endif
