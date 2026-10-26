#pragma once

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
        const std::array<double, 3>& joint_acceleration,
        const std::array<double, 3>& motor_torque,
        const std::array<double, 9>& mass_matrix,
        const std::array<double, 9>& jacobian,
        const std::array<double, 3>& bias_torque,
        const std::array<double, 3>& passive_torque,
        const std::array<double, 3>& friction_limit);

    bool valid() const {
        return valid_;
    }

    const std::array<double, 3>& force() const {
        return force_;
    }

    const std::array<double, 3>& torque() const {
        return filtered_torque_;
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

    bool valid_ = false;
    std::uint64_t measurement_ = 0;
    std::array<double, 3> force_{}, filtered_torque_{};
    double minimum_singular_value_ = 0;
};
}  // namespace qhrr05
