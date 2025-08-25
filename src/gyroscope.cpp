#include "sensorbox/gyroscope.hpp"

namespace sensorbox {

template class Gyroscope<2>;
template class Gyroscope<3>;
template class JsonLoadable<GyroscopeSchemaFilepath, sensorbox_schema_loader>;

}
