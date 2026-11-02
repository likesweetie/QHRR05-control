#include "motion_controllers/reaction_force_observer.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
constexpr double kTwoPi = 6.2831853071795864769;
template <std::size_t N> bool Finite(const std::array<double, N>& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
}
ReactionForceObserver::ReactionForceObserver(const std::string& path) {
    const auto node = YAML::LoadFile(path)["magnetic_touch"]["observer"];
    bandwidth_ = node["bandwidth_hz"].as<double>();
    regularization_ = node["regularization_m"].as<double>();
    singular_limit_ = node["minimum_singular_value_m"].as<double>();
    force_limit_ = node["maximum_estimate_n"].as<double>();
    friction_velocity_ = node["friction_velocity_rad_s"].as<double>();
    maximum_sample_gap_ = node["maximum_sample_gap_sec"].as<double>(.02);

    for (double value :
        {bandwidth_, regularization_, singular_limit_, force_limit_, maximum_sample_gap_})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error("GM observer parameters must be positive and finite");
    if (!std::isfinite(friction_velocity_) || friction_velocity_ <= 0)
        throw std::runtime_error(
            "magnetic_touch.observer.friction_velocity_rad_s must be positive and finite "
            "(tanh velocity scale, not a friction enable flag)");
    const auto harmonics = node["harmonic_compensation"];
    harmonic_enabled_ = harmonics && harmonics["enabled"].as<bool>(false);
    if (harmonic_enabled_) {
        const auto motor_path = std::filesystem::path(path).parent_path() /
            harmonics["motor_config"].as<std::string>();
        gear_ratio_ = YAML::LoadFile(motor_path.lexically_normal().string())["mit"]["gear_ratio"].as<double>();
        if (!std::isfinite(gear_ratio_) || gear_ratio_ <= 0)
            throw std::runtime_error("GM harmonic gear ratio must be positive and finite");
        if (const auto offset = harmonics["motor_zero_rad"]) {
            if (offset.IsScalar()) motor_zero_.fill(offset.as<double>());
            else {
                const auto values = offset.as<std::vector<double>>();
                if (values.size() != 3) throw std::runtime_error("GM harmonics need three motor zero offsets");
                std::copy(values.begin(), values.end(), motor_zero_.begin());
            }
        }
        const auto coefficients = harmonics["coefficients_nm"];
        if (!coefficients.IsSequence() || coefficients.size() != 3 || !Finite(motor_zero_))
            throw std::runtime_error("GM harmonics need three finite coefficient rows/offsets");
        for (int axis = 0; axis < 3; ++axis) {
            const auto values = coefficients[axis].as<std::vector<double>>();
            if (values.size() != 4)
                throw std::runtime_error("GM harmonic rows must be [sin21, cos21, sin36, cos36]");
            std::copy(values.begin(), values.end(), harmonic_coefficients_[axis].begin());
            if (!Finite(harmonic_coefficients_[axis]))
                throw std::runtime_error("GM harmonic coefficients must be finite");
        }
    }
    Reset();
}

void ReactionForceObserver::Reset() {
    measurement_ = 0;
    factor_valid_ = false;
    valid_ = false;
    momentum_initialized_ = false;
    residual_initialized_ = false;
    minimum_singular_value_ = 0;
    force_.fill(0);
    filtered_torque_.fill(0);
    compensated_torque_.fill(0);
    previous_momentum_.fill(0);
    previous_model_input_.fill(0);
}

void ReactionForceObserver::Update(std::uint64_t measurement_sequence,
    double measurement_dt,
    const std::array<double, 3>& joint_velocity,
    const std::array<double, 3>& motor_torque,
    const std::array<double, 9>& mass_matrix,
    const std::array<double, 9>& coriolis_matrix,
    const std::array<double, 9>& jacobian,
    const std::array<double, 3>& bias_torque,
    const std::array<double, 3>& passive_torque,
    const std::array<double, 3>& friction_limit,
    const std::array<double, 3>& joint_position) {
    if (!measurement_sequence || !std::isfinite(measurement_dt) || measurement_dt <= 0 ||
        !Finite(mass_matrix) || !Finite(coriolis_matrix) || !Finite(jacobian) ||
        !Finite(motor_torque) || !Finite(bias_torque) || !Finite(passive_torque) ||
        !Finite(friction_limit) || !Finite(joint_velocity) ||
        (harmonic_enabled_ && !Finite(joint_position))) {
        Reset();
        return;
    }
    if (measurement_sequence < measurement_)
        Reset();

    if (!factor_valid_ || factored_jacobian_ != jacobian) {
        factor_valid_ =
            solver_.Factor(jacobian, {singular_limit_, regularization_, regularization_, 10});
        factored_jacobian_ = jacobian;
    }
    if (!factor_valid_) {
        Reset();
        return;
    }
    minimum_singular_value_ = solver_.spectrum().minimum;

    if (minimum_singular_value_ < singular_limit_) {
        Reset();
        return;
    }

    if (measurement_sequence == measurement_)
        return;  // Never integrate the same observation twice.

    const auto samples = measurement_ ? measurement_sequence - measurement_ : 1;
    const double dt = measurement_dt * samples;
    if (!std::isfinite(dt) || dt > maximum_sample_gap_) {
        Reset();
        return;
    }

    std::array<double, 3> momentum{}, model_input{};
    for (int i = 0; i < 3; ++i) {
        model_input[i] = motor_torque[i] + passive_torque[i] - bias_torque[i] -
            friction_limit[i] * std::tanh(joint_velocity[i] / friction_velocity_);
        for (int j = 0; j < 3; ++j) {
            momentum[i] += mass_matrix[3 * i + j] * joint_velocity[j];
            model_input[i] += (coriolis_matrix[3 * i + j] + coriolis_matrix[3 * j + i]) * joint_velocity[j];
        }
    }
    if (!Finite(momentum) || !Finite(model_input)) {
        Reset();
        return;
    }
    if (!momentum_initialized_) {
        previous_momentum_ = momentum;
        previous_model_input_ = model_input;
        measurement_ = measurement_sequence;
        momentum_initialized_ = true;
        return;  // Prime p=M*dq; a nonzero entry velocity is not an impact.
    }
    const double alpha = -std::expm1(-kTwoPi * bandwidth_ * dt);
    for (int i = 0; i < 3; ++i) {
        const double innovation = momentum[i] - previous_momentum_[i] -
            .5 * (model_input[i] + previous_model_input_[i]) * dt;
        filtered_torque_[i] = (1 - alpha) * filtered_torque_[i] + alpha / dt * innovation;
    }
    previous_momentum_ = momentum;
    previous_model_input_ = model_input;
    measurement_ = measurement_sequence;
    residual_initialized_ = true;
    compensated_torque_ = filtered_torque_;
    if (harmonic_enabled_)
        for (int axis = 0; axis < 3; ++axis) {
            const double angle = joint_position[axis] * gear_ratio_ + motor_zero_[axis];
            if (!std::isfinite(angle)) { Reset(); return; }
            const double theta = std::remainder(angle, kTwoPi);
            const auto& h = harmonic_coefficients_[axis];
            compensated_torque_[axis] -= h[0] * std::sin(21 * theta) + h[1] * std::cos(21 * theta) +
                h[2] * std::sin(36 * theta) + h[3] * std::cos(36 * theta);
        }

    if (!Finite(compensated_torque_) || !solver_.SolveTranspose(compensated_torque_, force_)) {
        Reset();
        return;
    }
    double squared = 0;

    for (double value : force_)
        squared += value * value;

    valid_ = std::isfinite(squared) && squared <= force_limit_ * force_limit_;
    if (!valid_) force_.fill(0);
}
}  // namespace qhrr05
