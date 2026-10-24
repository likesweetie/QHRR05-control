#include <array>
#include "motion_controllers/homing_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace qhrr05 {
namespace {
std::array<double, 3> Joints(const YAML::Node& value, bool scalar = false) {
    std::array<double, 3> result;
    if (scalar && value.IsScalar()) result.fill(value.as<double>());
    else {
        const auto values = value.as<std::vector<double>>();
        if (values.size() != 3) throw std::runtime_error("expected three joint values");
        std::copy(values.begin(), values.end(), result.begin());
    }
    for (double v : result) if (!std::isfinite(v)) throw std::runtime_error("joint values must be finite");
    return result;
}
}

HomingController::HomingController(const YAML::Node& settings)
    : kp_(Joints(settings["kp"], true)), kd_(Joints(settings["kd"], true)),
      initial_(Joints(settings["homing"]["initial_position_rad"])),
      home_(Joints(settings["homing"]["position_rad"])),
      duration_(settings["homing"]["duration_sec"].as<double>()),
      bias_compensation_(settings["bias_compensation"].as<bool>()) {
    if (settings["robot"].as<std::string>() != "qhrr-05") throw std::runtime_error("only qhrr-05 is supported");
    if (!std::isfinite(duration_) || duration_ <= 0) throw std::runtime_error("homing duration must be positive and finite");
    for (int i = 0; i < 3; ++i)
        if (kp_[i] <= 0 || kd_[i] < 0) throw std::runtime_error("kp must be positive and kd nonnegative");
}

HomingReference HomingController::Reference(double sim_time) const {
    HomingReference ref;
    const double s = std::clamp(sim_time / duration_, 0.0, 1.0);
    const double blend = s*s*s*(10.0 + s*(-15.0 + 6.0*s));
    const double rate = 30.0*s*s*(1.0-s)*(1.0-s) / duration_;
    ref.holding = sim_time >= duration_;
    for (int i = 0; i < 3; ++i) {
        const double delta = home_[i] - initial_[i];
        ref.position[i] = initial_[i] + blend * delta;
        ref.velocity[i] = rate * delta;
    }
    return ref;
}

void HomingController::Compute(double time, const double position[3], const double velocity[3],
                               const double bias[3], double torque[3], double reference[3]) const {
    if (!std::isfinite(time) || time < 0) throw std::runtime_error("invalid simulation time");
    const auto ref = Reference(time);
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) || !std::isfinite(bias[i]))
            throw std::runtime_error("invalid joint input");
        reference[i] = ref.position[i];
        torque[i] = kp_[i]*(ref.position[i]-position[i]) + kd_[i]*(ref.velocity[i]-velocity[i]) +
                    (bias_compensation_ ? bias[i] : 0.0);
        if (!std::isfinite(torque[i])) throw std::runtime_error("non-finite torque command");
    }
}
}  // namespace qhrr05
