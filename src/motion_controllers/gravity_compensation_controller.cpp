#include "motion_controllers/gravity_compensation_controller.hpp"
#include "control/control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
GravityCompensationController::GravityCompensationController(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const bool torque_ref_as_position = ReadTorqueRefAsPosition(config);

    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto settings = config["gravity_compensation"];
    if (settings["virtual_inertia"])
        throw std::runtime_error(
            "gravity_compensation.virtual_inertia was removed; delete the obsolete section");
    if (settings["task_stiffness"])
        throw std::runtime_error(
            "gravity_compensation.task_stiffness was removed; gravity mode is pure torque compensation");
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
    friction_compensation_enabled_ = settings["friction_compensation"]["enabled"].as<bool>();
    read(settings["friction_compensation"]["coulomb_ratio"], coulomb_ratio_);
    torque_limit_ = ReadControlTorqueLimits(config);
    friction_velocity_ = settings["friction_compensation"]["velocity_scale_rad_s"].as<double>();

    if (!std::isfinite(alpha_) || alpha_ < 0 || alpha_ > 1)
        throw std::runtime_error("gravity alpha must be finite and in [0, 1]");

    for (int i = 0; i < 3; ++i) {
        if (kp_motor_[i] < 0 || kd_motor_[i] < 0)
            throw std::runtime_error("gravity motor kp and kd must be nonnegative");
        if (torque_ref_as_position && kp_motor_[i] == 0)
            throw std::runtime_error(
                "gravity_compensation.kp_motor must be positive when torque_reference_mode=torque_ref_as_position");
    }

    for (int i = 0; i < 3; ++i)
        if (coulomb_ratio_[i] < 0 || coulomb_ratio_[i] > 1 || torque_limit_[i] <= 0)
            throw std::runtime_error(
                "gravity ratios must be in [0,1]; torque limits must be positive");

    if (!std::isfinite(friction_velocity_) || friction_velocity_ <= 0)
        throw std::runtime_error("gravity friction velocity scale must be positive and finite");
}

void GravityCompensationController::Start(const double position[3]) {
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(position[i]))
            throw std::runtime_error("invalid gravity entry position");
    std::cout << "[gravity] coulomb compensation=[" << coulomb_ratio_[0] << ','
              << coulomb_ratio_[1] << ',' << coulomb_ratio_[2] << "]\n";
    std::cout << "[gravity] friction compensation="
              << (friction_compensation_enabled_ ? "ON" : "OFF") << '\n';
}

void GravityCompensationController::Compute(
    const RobotModelState& state, std::array<double, 3>& position_reference,
    std::array<double, 3>& torque) const {
    const auto& compensation = gravity_only_ ? state.gravity : state.bias;

    for (const auto& values :
        {state.joint_position, state.joint_velocity, compensation, state.friction})
        for (double value : values)
            if (!std::isfinite(value))
                throw std::runtime_error("invalid gravity joint/model input");

    for (int i = 0; i < 3; ++i) {
        if (state.friction[i] < 0)
            throw std::runtime_error("gravity friction model must be nonnegative");
        const double velocity = state.joint_velocity[i];
        const double dry =
            friction_compensation_enabled_
                ? coulomb_ratio_[i] * state.friction[i] * std::tanh(velocity / friction_velocity_)
                : 0;
        torque[i] = alpha_ * compensation[i] + dry;
        position_reference[i] = state.joint_position[i];

        if (!std::isfinite(torque[i]))
            throw std::runtime_error("non-finite torque command");
        torque[i] = std::clamp(torque[i], -torque_limit_[i], torque_limit_[i]);
    }
}
}  // namespace qhrr05
