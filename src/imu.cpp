#include "sensorbox/imu.hpp"

namespace sensorbox {

template class Imu<2>;
template class Imu<3>;
template class JsonLoadable<ImuSchemaFilepath, sensorbox_schema_loader>;

template class ImuMeasurement<2>;
template class ImuMeasurement<3>;

}
