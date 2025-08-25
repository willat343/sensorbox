#include "sensorbox/stiffness.hpp"

namespace sensorbox {

template Eigen::MatrixXd stiffness_from_config<Eigen::Dynamic>(const nlohmann::json&);
template Eigen::Matrix<double, 1, 1> stiffness_from_config<1>(const nlohmann::json&);
template Eigen::Matrix<double, 2, 2> stiffness_from_config<2>(const nlohmann::json&);
template Eigen::Matrix<double, 3, 3> stiffness_from_config<3>(const nlohmann::json&);
template Eigen::Matrix<double, 6, 6> stiffness_from_config<6>(const nlohmann::json&);

}
