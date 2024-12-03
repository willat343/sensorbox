#ifndef SENSORBOX_RANDOM_WALK_SENSOR_HPP
#define SENSORBOX_RANDOM_WALK_SENSOR_HPP

#include <Eigen/Core>

namespace sensorbox {

template<int DoF_>
class RandomWalkSensorProperties {
public:
    static constexpr int DoF = DoF_;
    using Stiffness = Eigen::Matrix<double, DoF, DoF>;

    /**
     * @brief Construct an instance of the class
     *
     * @param frequency
     * @param noise_density
     * @param bias_noise_density
     */
    explicit RandomWalkSensorProperties(const double frequency, const double noise_density,
            const double bias_noise_density);

    /**
     * @brief Bias (random walk) noise density (in sensor units * s^{-1} * Hz^{-1/2})
     *
     * @return double
     */
    double bias_noise_density() const;

    /**
     * @brief Sensor frequency (Hz)
     *
     * @return double
     */
    double frequency() const;

    /**
     * @brief Noise density (in sensor units * Hz^{-1/2})
     *
     * @return double
     */
    double noise_density() const;

    /**
     * @brief Sensor period (s)
     * 
     * @return double 
     */
    double period() const;

    /**
     * @brief Set the bias noise density
     *
     * @param bias_noise_density_
     */
    void set_bias_noise_density(const double bias_noise_density_);

    /**
     * @brief Set the frequency
     *
     * @param frequency_
     */
    void set_frequency(const double frequency_);

    /**
     * @brief Set the noise density
     *
     * @param noise_density_
     */
    void set_noise_density(const double noise_density_);

    /**
     * @brief Set the properties
     *
     * @param frequency_
     * @param noise_density_
     */
    void set_properties(const double frequency_, const double noise_density_);

    /**
     * @brief Set the properties
     *
     * @param frequency_
     * @param noise_density_
     * @param bias_noise_density_
     */
    void set_properties(const double frequency_, const double noise_density_, const double bias_noise_density_);

    /**
     * @brief Get the stiffness matrix
     *
     * @return const Stiffness&
     */
    const Stiffness& stiffness() const;

private:
    /**
     * @brief Update the stiffness matrix so that it can be computed once whenever the sensor properties change.
     *
     */
    void update_stiffness();

    // Frequency (Hz)
    double frequency__;

    // Noise Density (unspecified units)
    double noise_density__;

    // Bias (Random Walk) Noise Density (unspecified units)
    double bias_noise_density__;

    // Stiffness matrix
    Stiffness stiffness_;
};

}

#include "sensorbox/impl/random_walk_sensor.hpp"

#endif
