#include "sensorbox/accelerometer.hpp"

namespace sensorbox {

template class Accelerometer<2>;
template class Accelerometer<3>;
template class JsonLoadable<AccelerometerSchemaFilepath, sensorbox_schema_loader>;

}
