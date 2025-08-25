#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

template class RandomWalkSensor<1>;
template class RandomWalkSensor<2>;
template class RandomWalkSensor<3>;
template class JsonLoadable<RandomWalkSensorSchemaFilepath, sensorbox_schema_loader>;

}
