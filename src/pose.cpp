#include "sensorbox/pose.hpp"

namespace sensorbox {

template class PoseSensor<2>;
template class PoseSensor<3>;
template class JsonLoadable<PoseSensorSchemaFilepath, sensorbox_schema_loader>;

template class PoseMeasurement<2>;
template class PoseMeasurement<3>;

template class PoseMeasurements<2>;
template class PoseMeasurements<3>;

}
