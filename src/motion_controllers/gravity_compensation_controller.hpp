#pragma once

#include <array>
#include <string>
#include <cstdint>
#include "robot_model/backend.hpp"

namespace qhrr05 {
// Encoder-derived acceleration and measured motor effort; no simulator ground truth.
struct GravityFeedback {
    std::array<double, 3> acceleration{}, motor_torque{};
    std::uint64_t sequence = 0;
    double dt = 0;
};

class GravityCompensationController {
public:
    explicit GravityCompensationController(const std::string& config_path);
    void Start(const double position[3]);

    const std::array<double, 3>& kp_motor() const {
        return kp_motor_;
    }

    const std::array<double, 3>& kd_motor() const {
        return kd_motor_;
    }

    void Compute(const RobotModelState& state,
        const GravityFeedback& feedback,
        double torque[3],
        double reference[3]);

    const std::array<double, 3>& inertia_assist() const {
        return inertia_assist_;
    }

private:
    std::array<double, 3> kp_motor_, kd_motor_, task_stiffness_, desired_{};
    std::array<double, 3> damping_ratio_, coulomb_ratio_, torque_limit_, assist_limit_;
    std::array<double, 3> filtered_external_{}, inertia_assist_{};
    double alpha_;
    bool gravity_only_;
    double friction_velocity_, inertia_scale_, observer_hz_, deadband_, ramp_sec_, maximum_speed_;
    bool inertia_enabled_;
    std::uint64_t measurement_ = 0;
    double observer_time_ = 0;
};
}  // namespace qhrr05
