#ifndef SENSORBOX_GYROSCOPE_HPP
#define SENSORBOX_GYROSCOPE_HPP

#include <nlohmann/json.hpp>

#include "sensorbox/json_loadable.hpp"
#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

/**
 * @brief Gyroscope properties. Sensor units are rad * s^{-1}.
 *
 * @tparam D_ dimension (e.g. 3 for 3D)
 */
template<int D_>
class Gyroscope : public RandomWalkSensor<D_*(D_ - 1) / 2>,
                  public JsonLoadable<GyroscopeSchemaFilepath, sensorbox_schema_loader> {
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
    static constexpr int DoF = D * (D - 1) / 2;

    /**
     * @brief Construct an instance of the class from a json config with RandomWalkSensor structure.
     *
     * @param config
     */
    explicit Gyroscope(const nlohmann::json& config);

    /**
     * @brief Construct an instance of the class
     *
     * @param frequency
     * @param noise_density
     * @param bias_noise_density
     * @param initial_noise
     */
    explicit Gyroscope(const double frequency, const double noise_density, const double bias_noise_density,
            const double initial_noise);
};

}

#include "sensorbox/impl/gyroscope.hpp"

#endif
