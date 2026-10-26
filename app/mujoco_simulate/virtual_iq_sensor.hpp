#pragma once

#include <cstdint>
#include <random>
#include <string>

namespace qhrr05 {
class VirtualIqSensor {
public:
    VirtualIqSensor(const std::string& config_path, int axis);
    double Sample(double iq_true_a, double electrical_angle_rad, double dt_sec);
    void Reset();

    bool enabled() const {
        return enabled_;
    }

private:
    bool enabled_;
    double gain_, offset_, noise_std_, ripple_amplitude_, ripple_harmonic_, ripple_phase_;
    double bandwidth_, quantization_, current_limit_;
    std::uint32_t seed_;
    std::mt19937 generator_;
    std::normal_distribution<double> normal_{0, 1};
    double filtered_a_ = 0;
    bool initialized_ = false;
};
}  // namespace qhrr05
