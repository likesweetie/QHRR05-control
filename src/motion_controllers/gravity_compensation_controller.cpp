#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/numerics/math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <iostream>
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
    read(settings["damping_ratio"], damping_ratio_);
    read(settings["friction_compensation"]["coulomb_ratio"], coulomb_ratio_);
    read(settings["torque_limit_nm"], torque_limit_);
    friction_velocity_ = settings["friction_compensation"]["velocity_scale_rad_s"].as<double>();
    const auto inertia = settings["virtual_inertia"];
    inertia_enabled_ = inertia["enabled"].as<bool>();
    inertia_scale_ = inertia["scale"].as<double>();
    observer_hz_ = inertia["observer_hz"].as<double>();
    deadband_ = inertia["external_torque_deadband_nm"].as<double>();
    ramp_sec_ = inertia["ramp_sec"].as<double>();
    maximum_speed_ = inertia["maximum_velocity_rad_s"].as<double>();
    read(inertia["assist_limit_nm"], assist_limit_);

    if (!std::isfinite(alpha_) || alpha_ < 0 || alpha_ > 1)
        throw std::runtime_error("gravity alpha must be finite and in [0, 1]");

    for (int i = 0; i < 3; ++i)
        if (kp_motor_[i] <= 0 || kd_motor_[i] < 0 || task_stiffness_[i] < 0)
            throw std::runtime_error(
                "motor kp must be positive; kd and task stiffness must be nonnegative");

    for (int i = 0; i < 3; ++i)
        if (damping_ratio_[i] < 0 || damping_ratio_[i] > 1 || coulomb_ratio_[i] < 0 ||
            coulomb_ratio_[i] > 1 || torque_limit_[i] <= 0 || assist_limit_[i] <= 0 ||
            assist_limit_[i] > torque_limit_[i])
            throw std::runtime_error(
                "gravity ratios must be in [0,1]; torque/assist limits must be positive and assist <= total");

    for (double value :
        {friction_velocity_, inertia_scale_, observer_hz_, ramp_sec_, maximum_speed_})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error(
                "gravity friction/inertia parameters must be positive and finite");
    if (inertia_scale_ < .25 || inertia_scale_ > 1 || !std::isfinite(deadband_) || deadband_ < 0)
        throw std::runtime_error("virtual inertia scale must be in [0.25,1], deadband nonnegative");
    if (inertia_enabled_ && (gravity_only_ || alpha_ != 1))
        throw std::runtime_error("virtual inertia requires gravity_only=false and alpha=1");
}

void GravityCompensationController::Start(const double position[3]) {
    std::copy_n(position, 3, desired_.begin());
    filtered_external_.fill(0);
    inertia_assist_.fill(0);
    measurement_ = 0;
    observer_time_ = 0;
    std::cout << "[gravity] damping remainder=[" << damping_ratio_[0] << ',' << damping_ratio_[1]
              << ',' << damping_ratio_[2] << "] coulomb compensation=[" << coulomb_ratio_[0] << ','
              << coulomb_ratio_[1] << ',' << coulomb_ratio_[2]
              << "] virtual inertia=" << (inertia_enabled_ ? "ON" : "OFF")
              << " scale=" << inertia_scale_ << '\n';
}

void GravityCompensationController::Compute(const RobotModelState& state,
    const GravityFeedback& feedback,
    double torque[3],
    double reference[3]) {
    const auto& compensation = gravity_only_ ? state.gravity : state.bias;

    for (const auto& values :
        {state.joint_position, state.joint_velocity, compensation, state.damping, state.friction})
        for (double value : values)
            if (!std::isfinite(value))
                throw std::runtime_error("invalid gravity joint/model input");

    if (inertia_enabled_) {
        if (!feedback.sequence || feedback.sequence < measurement_ || !std::isfinite(feedback.dt) ||
            feedback.dt <= 0 || feedback.dt > .05)
            throw std::runtime_error(
                "virtual inertia requires ordered fresh measurements with dt in (0,0.05]");
        for (const auto& values :
            {feedback.acceleration, feedback.motor_torque, state.passive, state.bias})
            for (double value : values)
                if (!std::isfinite(value))
                    throw std::runtime_error("invalid virtual inertia feedback");
        for (double value : state.mass)
            if (!std::isfinite(value))
                throw std::runtime_error("invalid virtual inertia mass matrix");

        if (feedback.sequence != measurement_) {
            // dt is elapsed time since the previous fresh sample, not sequence count * dt.
            const double blend = -std::expm1(-2 * 3.141592653589793 * observer_hz_ * feedback.dt);
            const auto inertial = numeric::Multiply(state.mass, feedback.acceleration);
            for (int i = 0; i < 3; ++i) {
                const double dry =
                    state.friction[i] * std::tanh(state.joint_velocity[i] / friction_velocity_);
                const double external =
                    inertial[i] + state.bias[i] - state.passive[i] + dry - feedback.motor_torque[i];
                if (!std::isfinite(external))
                    throw std::runtime_error("non-finite external torque estimate");
                filtered_external_[i] += blend * (external - filtered_external_[i]);
            }
            observer_time_ += feedback.dt;
            measurement_ = feedback.sequence;
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (state.damping[i] < 0 || state.friction[i] < 0)
            throw std::runtime_error("gravity friction/damping model must be nonnegative");
        const double velocity = state.joint_velocity[i];
        const double tau_task = task_stiffness_[i] * (desired_[i] - state.joint_position[i]);
        const double viscous = (1 - damping_ratio_[i]) * state.damping[i] * velocity;
        const double dry =
            coulomb_ratio_[i] * state.friction[i] * std::tanh(velocity / friction_velocity_);
        inertia_assist_[i] = 0;
        if (inertia_enabled_) {
            const double external = std::copysign(
                std::max(0.0, std::abs(filtered_external_[i]) - deadband_), filtered_external_[i]);
            const double ramp = std::min(1.0, observer_time_ / ramp_sec_);
            inertia_assist_[i] = std::clamp(
                ramp * (1 / inertia_scale_ - 1) * external, -assist_limit_[i], assist_limit_[i]);
            // Taper assistance that accelerates an already fast joint; braking remains available.
            if (inertia_assist_[i] * velocity > 0)
                inertia_assist_[i] *=
                    std::clamp(2 * (1 - std::abs(velocity) / maximum_speed_), 0.0, 1.0);
        }
        torque[i] = alpha_ * compensation[i] + tau_task + viscous + dry - kd_motor_[i] * velocity +
                    inertia_assist_[i];
        reference[i] = desired_[i];

        if (!std::isfinite(torque[i]))
            throw std::runtime_error("non-finite torque command");
        torque[i] = std::clamp(torque[i], -torque_limit_[i], torque_limit_[i]);
    }
}
}  // namespace qhrr05
