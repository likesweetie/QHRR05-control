#include <array>
#include "motion_controllers/homing_controller.hpp"
#include "control/control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
std::array<double, 3> Joints(const YAML::Node& value, bool scalar = false) {
    std::array<double, 3> result;

    if (scalar && value.IsScalar())
        result.fill(value.as<double>());
    else {
        const auto values = value.as<std::vector<double>>();

        if (values.size() != 3)
            throw std::runtime_error("expected three joint values");

        std::copy(values.begin(), values.end(), result.begin());
    }

    for (double v : result)
        if (!std::isfinite(v))
            throw std::runtime_error("joint values must be finite");

    return result;
}
}

HomingController::HomingController(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const bool torque_ref_as_position = ReadTorqueRefAsPosition(config);

    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto settings = config["homing"];
    if (settings["torque_control"])
        torque_control_ = settings["torque_control"].as<bool>();
    differentiate_position_ = settings["differentiate_position"].as<bool>(false);
    if (const auto lpf = config["homing_velocity_lpf"]) {
        velocity_lpf_enabled_ = lpf["enabled"].as<bool>();
        velocity_lpf_cutoff_hz_ = lpf["cutoff_hz"].as<double>();
    }
    if (velocity_lpf_enabled_ &&
        (!std::isfinite(velocity_lpf_cutoff_hz_) || velocity_lpf_cutoff_hz_ <= 0))
        throw std::runtime_error("homing_velocity_lpf.cutoff_hz must be positive and finite when enabled");
    torque_limit_ = ReadControlTorqueLimits(config);
    kp_ = Joints(settings["kp"], true);
    kd_ = Joints(settings["kd"], true);
    initial_ = Joints(settings["initial_position_rad"]);
    home_ = Joints(settings["position_rad"]);
    duration_ = settings["duration_sec"].as<double>();

    if (!std::isfinite(duration_) || duration_ <= 0)
        throw std::runtime_error("homing duration must be positive and finite");

    for (int i = 0; i < 3; ++i) {
        if (kp_[i] < 0 || kd_[i] < 0)
            throw std::runtime_error("homing kp and kd must be nonnegative");
        if (torque_ref_as_position && kp_[i] == 0)
            throw std::runtime_error(
                "homing.kp must be positive when torque_reference_mode=torque_ref_as_position");
    }
}

void HomingController::Start(double time, const double position[3]) {
    start_time_ = time;
    std::copy_n(position, 3, initial_.begin());
    initialized_.fill(false);
    velocity_.fill(0);
}

HomingReference HomingController::Reference(double sim_time) const {
    HomingReference ref;
    const double s = std::clamp((sim_time - start_time_) / duration_, 0.0, 1.0);
    const double blend = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
    const double rate = 30.0 * s * s * (1.0 - s) * (1.0 - s) / duration_;

    ref.holding = sim_time - start_time_ >= duration_;

    for (int i = 0; i < 3; ++i) {
        const double delta = home_[i] - initial_[i];
        ref.position[i] = initial_[i] + blend * delta;
        ref.velocity[i] = rate * delta;
    }

    return ref;
}

void HomingController::Compute(double time,
    const double position[3],
    const double velocity[3],
    const std::array<std::uint64_t, 3>& sequence,
    const std::array<std::uint64_t, 3>& sample_ns,
    std::array<double, 3>& position_reference,
    std::array<double, 3>& torque) {
    if (!std::isfinite(time) || time < 0)
        throw std::runtime_error("invalid simulation time");

    const auto ref = Reference(time);

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]))
            throw std::runtime_error("invalid joint input");

        position_reference[i] = ref.position[i];
        if (torque_control_) {
            if (!differentiate_position_ && !velocity_lpf_enabled_) {
                velocity_[i] = velocity[i];
            } else if (!initialized_[i] || sequence[i] != sequence_[i] || sample_ns[i] < sample_ns_[i]) {
                const bool reset = !initialized_[i] || sequence[i] < sequence_[i] || sample_ns[i] <= sample_ns_[i];
                if (reset) {
                    // The first position sample only seeds the difference history.
                    velocity_[i] = differentiate_position_ ? 0 : velocity[i];
                } else {
                    const double dt = (sample_ns[i] - sample_ns_[i]) / 1e9;
                    const double raw = differentiate_position_
                        ? (position[i] - previous_position_[i]) / dt : velocity[i];
                    if (!std::isfinite(raw))
                        throw std::runtime_error("non-finite homing velocity estimate");
                    const double alpha = velocity_lpf_enabled_
                        ? -std::expm1(-2 * 3.14159265358979323846 * velocity_lpf_cutoff_hz_ * dt) : 1;
                    velocity_[i] += alpha * (raw - velocity_[i]);
                }
                previous_position_[i] = position[i];
                sequence_[i] = sequence[i];
                sample_ns_[i] = sample_ns[i];
                initialized_[i] = true;
            }
            if (!std::isfinite(velocity_[i]))
                throw std::runtime_error("non-finite filtered homing velocity");
        }
        // Optional host PD uses the selected measured/estimated velocity, never ref.velocity.
        // TaskController disables MIT P/D; RMD keeps its configured damping.
        torque[i] = torque_control_
            ? std::clamp(kp_[i] * (ref.position[i] - position[i]) - kd_[i] * velocity_[i],
                  -torque_limit_[i], torque_limit_[i])
            : 0;
    }
}
}  // namespace qhrr05
