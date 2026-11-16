#include "sensorbox/position.hpp"

namespace sensorbox {

template class PositionSensor<2>;
template class PositionSensor<3>;
template class JsonLoadable<PositionSensorSchemaFilepath, sensorbox_schema_loader>;

template class PositionMeasurement<2>;
template class PositionMeasurement<3>;

}
