#ifndef SENSORBOX_UNARY_HPP
#define SENSORBOX_UNARY_HPP

#include <chrono>
#include <string>

namespace sensorbox {

class UnaryMeasurement {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::nanoseconds;
    using Timestamp = std::chrono::time_point<Clock, Duration>;

    /**
     * @brief Construct a Unary Measurement. The timestamp is initialised to zero and frames left empty.
     * 
     */
    explicit UnaryMeasurement();

    /**
     * @brief Construct a Unary Measurement. The child_frame_ is set to be the same as frame_.
     * 
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is represented
     */
    explicit UnaryMeasurement(const Timestamp& timestamp_, const std::string& frame_);

    /**
     * @brief Construct a unary measurement.
     * 
     * @param timestamp_ timestamp
     * @param frame_ reference frame in which the measurement is represented
     * @param child_frame_ reference frame the measurement represents
     */
    explicit UnaryMeasurement(const Timestamp& timestamp_, const std::string& frame_, const std::string& child_frame_);

    const std::string& child_frame() const;
    
    std::string& child_frame();

    const std::string& frame() const;
    
    std::string& frame();

    const Timestamp& timestamp() const;
    
    Timestamp& timestamp();

private:
    Timestamp timestamp_;
    std::string frame_;
    std::string child_frame_;
};

}

#endif
