#ifndef SENSORBOX_ACCELEROMETER_HPP
#define SENSORBOX_ACCELEROMETER_HPP

#include <nlohmann/json.hpp>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

/**
 * @brief Accelerometer properties. Sensor units are m * s^{-2}.
 *
 * @tparam D_ dimension (e.g. 3 for 3D)
 */
template<int D_>
class Accelerometer : public RandomWalkSensor<D_>,
                      public JsonLoadable<AccelerometerSchemaFilepath, sensorbox_schema_loader> {
public:
    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Degrees of Freedom (DoF)
     *
     */
    static constexpr int DoF = D;

    /**
     * @brief Construct an instance of the class from a json config with RandomWalkSensor structure.
     *
     * @param config
     * @param validate
     */
    explicit Accelerometer(const nlohmann::json& config, const bool validate = true);

    /**
     * @brief Construct an instance of the class
     *
     * @param frequency
     * @param noise_density
     * @param bias_noise_density
     * @param initial_noise
     */
    explicit Accelerometer(const double frequency, const double noise_density, const double bias_noise_density,
            const double initial_noise);
};

}

#include "sensorbox/impl/accelerometer.hpp"

#endif
