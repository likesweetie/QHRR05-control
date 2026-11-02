#pragma once

#include <array>
#include <cstdint>

#include <string>

namespace qhrr05 {
struct HomingReference {
    std::array<double, 3> position{}, velocity{};
    bool holding{false};
};

class HomingController {
public:
    explicit HomingController(const std::string& config_path);
    void Start(double time, const double position[3]);
    HomingReference Reference(double sim_time) const;

    const std::array<double, 3>& kp() const {
        return kp_;
    }

    const std::array<double, 3>& kd() const {
        return kd_;
    }

    bool torque_control() const {
        return torque_control_;
    }

    void Compute(double time,
        const double position[3],
        const double velocity[3],
        const std::array<std::uint64_t, 3>& sequence,
        const std::array<std::uint64_t, 3>& sample_ns,
        std::array<double, 3>& position_reference,
        std::array<double, 3>& torque);

private:
    std::array<double, 3> kp_, kd_, initial_, home_, torque_limit_;
    double duration_;
    double start_time_ = 0;
    bool torque_control_ = false;
    bool differentiate_position_ = false, velocity_lpf_enabled_ = false;
    double velocity_lpf_cutoff_hz_ = 20;
    std::array<double, 3> previous_position_{}, velocity_{};
    std::array<std::uint64_t, 3> sequence_{}, sample_ns_{};
    std::array<bool, 3> initialized_{};
};
}  // namespace qhrr05
