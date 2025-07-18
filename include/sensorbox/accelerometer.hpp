#ifndef SENSORBOX_ACCELEROMETER_HPP
#define SENSORBOX_ACCELEROMETER_HPP

#include "sensorbox/random_walk_sensor.hpp"

namespace sensorbox {

/**
 * @brief Accelerometer properties. Sensor units are m * s^{-2}.
 *
 * @tparam D_ dimension (e.g. 3 for 3D)
 */
template<int D_>
class Accelerometer : public RandomWalkSensor<D_> {
public:
    /**
     * @brief Dimension D
     *
     */
    static constexpr int D = D_;

    /**
     * @brief Construct an instance of the class from a json config with RandomWalkSensor structure.
     *
     * @param config
     */
    explicit Accelerometer(const nlohmann::json& config);

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

#endif
