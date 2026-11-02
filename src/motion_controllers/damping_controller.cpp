#include "motion_controllers/damping_controller.hpp"
#include "control/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
DampingController::DampingController(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const auto value = config["damping"]["gain_nm_s_rad"];
    if (value.IsScalar()) {
        gain_.fill(value.as<double>());
    } else {
        const auto values = value.as<std::vector<double>>();
        if (values.size() != 3)
            throw std::runtime_error("damping.gain_nm_s_rad needs three joint values");
        std::copy(values.begin(), values.end(), gain_.begin());
    }
    for (double gain : gain_)
        if (!std::isfinite(gain) || gain < 0)
            throw std::runtime_error("damping gain must be finite and nonnegative");
    limit_ = ReadControlTorqueLimits(config);
}

void DampingController::Reset() {
    initialized_.fill(false);
    velocity_.fill(0);
}

void DampingController::Compute(const std::array<double, 3>& position,
    const std::array<std::uint64_t, 3>& sequence,
    const std::array<std::uint64_t, 3>& sample_ns,
    std::array<double, 3>& position_reference,
    std::array<double, 3>& torque) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(position[i]))
            throw std::runtime_error("non-finite damping position sample");
        if (!initialized_[i] || sequence[i] != sequence_[i] || sample_ns[i] < sample_ns_[i]) {
            if (initialized_[i] && sample_ns[i] > sample_ns_[i]) {
                const double dt = (sample_ns[i] - sample_ns_[i]) / 1e9;
                const double delta = position[i] - previous_position_[i];
                const double raw_velocity = delta / dt;
                if (!std::isfinite(raw_velocity))
                    throw std::runtime_error("non-finite differentiated damping velocity");
                // First-order 30 Hz LPF, updated only on a fresh MIT position sample.
                const double alpha = -std::expm1(-2 * 3.141592653589793 * 20.0 * dt);
                velocity_[i] += alpha * (raw_velocity - velocity_[i]);
            } else {
                velocity_[i] = 0;
            }
            if (!std::isfinite(velocity_[i]))
                throw std::runtime_error("non-finite filtered damping velocity");
            previous_position_[i] = position[i];
            sequence_[i] = sequence[i];
            sample_ns_[i] = sample_ns[i];
            initialized_[i] = true;
        }
        position_reference[i] = position[i];
        torque[i] = std::clamp(-gain_[i] * velocity_[i], -limit_[i], limit_[i]);
    }
}
}  // namespace qhrr05
