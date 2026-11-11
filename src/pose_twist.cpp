#include "sensorbox/pose_twist.hpp"

namespace sensorbox {

template class PoseTwistMeasurement<2>;
template class PoseTwistMeasurement<3>;
template class PoseTwistWithCovarianceMeasurement<2>;
template class PoseTwistWithCovarianceMeasurement<3>;

}
