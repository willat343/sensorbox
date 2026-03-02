#include "sensorbox/position.hpp"

namespace sensorbox {

template class DirectPositionSensor<2>;
template class DirectPositionSensor<3>;
template class JsonLoadable<DirectPositionSensorSchemaFilepath, sensorbox_schema_loader>;

template class PositionMeasurement<2>;
template class PositionMeasurement<3>;

}
