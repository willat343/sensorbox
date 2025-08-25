#include "sensorbox/pose.hpp"

namespace sensorbox {

template class DirectPoseSensor<2>;
template class DirectPoseSensor<3>;
template class JsonLoadable<DirectPoseSensorSchemaFilepath, sensorbox_schema_loader>;

template class PoseMeasurement<2>;
template class PoseMeasurement<3>;

}
