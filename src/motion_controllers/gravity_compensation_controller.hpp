#pragma once

#include <array>
#include <string>
#include "robot_model/backend.hpp"

namespace qhrr05 {
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

    void Compute(const RobotModelState& state, double torque[3], double reference[3]) const;

private:
    std::array<double, 3> kp_motor_, kd_motor_, task_stiffness_, desired_{};
    std::array<double, 3> damping_ratio_, coulomb_ratio_, torque_limit_;
    double alpha_;
    bool gravity_only_;
    bool friction_compensation_enabled_;
    double friction_velocity_;
};
}  // namespace qhrr05
