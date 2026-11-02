#pragma once

#include "motion_controllers/numerics/math.hpp"
#include <array>
#include <cstdint>
#include <string>

namespace qhrr05 {
class ReactionForceObserver {
public:
    explicit ReactionForceObserver(const std::string& config_path);
    void Reset();
    void Update(std::uint64_t measurement_sequence,
        double measurement_dt,
        const std::array<double, 3>& joint_velocity,
        const std::array<double, 3>& motor_torque,
        const std::array<double, 9>& mass_matrix,
        const std::array<double, 9>& coriolis_matrix,
        const std::array<double, 9>& jacobian,
        const std::array<double, 3>& bias_torque,
        const std::array<double, 3>& passive_torque,
        const std::array<double, 3>& friction_limit,
        const std::array<double, 3>& joint_position);

    bool valid() const {
        return valid_;
    }
    bool warming_up() const { return momentum_initialized_ && !residual_initialized_; }

    const std::array<double, 3>& force() const {
        return force_;
    }

    const std::array<double, 3>& torque() const {
        return compensated_torque_;
    }

    double minimumSingularValue() const {
        return minimum_singular_value_;
    }

private:
    double bandwidth_;
    double regularization_;
    double singular_limit_;
    double force_limit_;
    double friction_velocity_;
    double maximum_sample_gap_ = .02;
    bool momentum_initialized_ = false, residual_initialized_ = false, harmonic_enabled_ = false;
    double gear_ratio_ = 1;
    std::array<double, 3> motor_zero_{};
    std::array<std::array<double, 4>, 3> harmonic_coefficients_{};

    bool valid_ = false;
    std::uint64_t measurement_ = 0;
    std::array<double, 3> force_{}, filtered_torque_{};
    std::array<double, 3> compensated_torque_{}, previous_momentum_{}, previous_model_input_{};
    double minimum_singular_value_ = 0;
    numeric::JacobianSolver solver_;
    numeric::Mat3 factored_jacobian_{};
    bool factor_valid_ = false;
};
}  // namespace qhrr05
