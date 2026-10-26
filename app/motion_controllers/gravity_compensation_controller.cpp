#include "gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
GravityCompensationController::GravityCompensationController(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);

    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto settings = config["gravity_compensation"];
    alpha_ = settings["alpha"].as<double>();
    gravity_only_ = settings["gravity_only"].as<bool>();

    const auto read = [](const YAML::Node& value, std::array<double, 3>& result) {
        if (value.IsScalar())
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
    };

    read(settings["kp_motor"], kp_motor_);
    read(settings["kd_motor"], kd_motor_);
    read(settings["task_stiffness"], task_stiffness_);

    if (!std::isfinite(alpha_) || alpha_ < 0 || alpha_ > 1)
        throw std::runtime_error("gravity alpha must be finite and in [0, 1]");

    for (int i = 0; i < 3; ++i)
        if (kp_motor_[i] <= 0 || kd_motor_[i] < 0 || task_stiffness_[i] < 0)
            throw std::runtime_error(
                "motor kp must be positive; kd and task stiffness must be nonnegative");
}

void GravityCompensationController::Start(const double position[3]) {
    std::copy_n(position, 3, desired_.begin());
}

void GravityCompensationController::Compute(const double position[3],
    const double velocity[3],
    const double gravity[3],
    const double bias[3],
    double torque[3],
    double reference[3]) const {
    const double* compensation = gravity_only_ ? gravity : bias;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) ||
            !std::isfinite(compensation[i]))
            throw std::runtime_error("invalid joint input");

        const double tau_task = task_stiffness_[i] * (desired_[i] - position[i]);
        const double q_surrogate = (alpha_ * compensation[i] + tau_task) / kp_motor_[i];
        const double q_command = position[i] + q_surrogate;

        // The XML uses torque motors. Explicitly simulate the inner position servo;
        // never send the equivalent angle directly to a torque-input actuator.
        torque[i] = kp_motor_[i] * (q_command - position[i]) - kd_motor_[i] * velocity[i];
        reference[i] = desired_[i];

        if (!std::isfinite(torque[i]))
            throw std::runtime_error("non-finite torque command");
    }
}
}  // namespace qhrr05
