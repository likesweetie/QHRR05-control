#include "motion_controllers/reaction_force_observer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
ReactionForceObserver::ReactionForceObserver(const std::string& path) {
    const auto node = YAML::LoadFile(path)["magnetic_touch"]["observer"];
    bandwidth_ = node["bandwidth_hz"].as<double>();
    regularization_ = node["regularization_m"].as<double>();
    singular_limit_ = node["minimum_singular_value_m"].as<double>();
    force_limit_ = node["maximum_estimate_n"].as<double>();
    friction_velocity_ = node["friction_velocity_rad_s"].as<double>();

    for (double value :
        {bandwidth_, regularization_, singular_limit_, force_limit_, friction_velocity_})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error("RFOB parameters must be positive and finite");
}

void ReactionForceObserver::Reset() {
    measurement_ = 0;
    factor_valid_ = false;
    valid_ = false;
    force_.fill(0);
    filtered_torque_.fill(0);
}

void ReactionForceObserver::Update(std::uint64_t measurement_sequence,
    double measurement_dt,
    const std::array<double, 3>& joint_velocity,
    const std::array<double, 3>& joint_acceleration,
    const std::array<double, 3>& motor_torque,
    const std::array<double, 9>& mass_matrix,
    const std::array<double, 9>& jacobian,
    const std::array<double, 3>& bias_torque,
    const std::array<double, 3>& passive_torque,
    const std::array<double, 3>& friction_limit) {
    for (const auto& values : {mass_matrix, jacobian})
        for (double value : values)
            if (!std::isfinite(value)) {
                valid_ = false;
                return;
            }

    for (const auto& values : {joint_acceleration,
             motor_torque,
             bias_torque,
             passive_torque,
             friction_limit,
             joint_velocity})
        for (double value : values)
            if (!std::isfinite(value)) {
                valid_ = false;
                return;
            }

    if (!factor_valid_ || factored_jacobian_ != jacobian) {
        factor_valid_ =
            solver_.Factor(jacobian, {singular_limit_, regularization_, regularization_, 10});
        factored_jacobian_ = jacobian;
    }
    if (!factor_valid_) {
        valid_ = false;
        return;
    }
    minimum_singular_value_ = solver_.spectrum().minimum;

    if (minimum_singular_value_ < singular_limit_) {
        valid_ = false;
        return;
    }

    if (!measurement_sequence || !std::isfinite(measurement_dt) || measurement_dt <= 0) {
        valid_ = false;
        return;
    }

    if (measurement_sequence == measurement_)
        return;  // No duplicate differentiation/filter update.

    if (measurement_sequence < measurement_)
        Reset();

    const auto samples = measurement_ ? measurement_sequence - measurement_ : 1;
    const double dt = measurement_dt * samples;
    const double alpha = -std::expm1(-2 * 3.141592653589793 * bandwidth_ * dt);

    for (int i = 0; i < 3; ++i) {
        double residual = bias_torque[i] - motor_torque[i] - passive_torque[i];

        for (int j = 0; j < 3; ++j)
            residual += mass_matrix[3 * i + j] * joint_acceleration[j];

        // Explicit smooth dry-friction MODEL, never actual constraint force.
        residual += friction_limit[i] * std::tanh(joint_velocity[i] / friction_velocity_);
        filtered_torque_[i] += alpha * (residual - filtered_torque_[i]);
    }

    measurement_ = measurement_sequence;

    if (!solver_.SolveTranspose(filtered_torque_, force_)) {
        valid_ = false;
        return;
    }
    double squared = 0;

    for (double value : force_)
        squared += value * value;

    valid_ = std::isfinite(squared) && squared <= force_limit_ * force_limit_;
}
}  // namespace qhrr05
