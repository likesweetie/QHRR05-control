#pragma once

#include "motion_controllers/reaction_force_observer.hpp"
#include "ipc/control_types.hpp"
#include <string>

namespace qhrr05 {
class MagneticTouchController {
public:
    explicit MagneticTouchController(const std::string& config_path);

    const std::array<double, 3>& kp_motor() const {
        return kp_motor_;
    }

    const std::array<double, 3>& kd_motor() const {
        return kd_motor_;
    }

    void Start(double time, const std::array<double, 3>& toe_position);
    void Compute(double time,
        const std::array<double, 3>& joint_position,
        const std::array<double, 3>& joint_velocity,
        const std::array<double, 3>& joint_acceleration,
        const std::array<double, 3>& motor_torque,
        const std::array<double, 3>& gravity_torque,
        const std::array<double, 3>& bias_torque,
        const std::array<double, 3>& passive_torque,
        const std::array<double, 3>& friction_limit,
        const std::array<double, 9>& mass_matrix,
        const std::array<double, 9>& jacobian,
        const std::array<double, 3>& toe_position,
        const std::array<double, 3>& joint_min,
        const std::array<double, 3>& joint_max,
        std::uint64_t measurement_sequence,
        double measurement_dt,
        double target_force,
        std::array<double, 3>& torque,
        std::array<double, 3>& reference);

    const MagneticTelemetry& telemetry() const {
        return telemetry_;
    }

private:
    void Transition(MagneticPhase phase, MagneticReason reason, double time);
    void Release(MagneticReason reason, double time);

    ReactionForceObserver observer_;
    MagneticTelemetry telemetry_{};

    std::array<double, 3> damping_;
    std::array<double, 3> kp_motor_, kd_motor_;
    std::array<double, 3> torque_limit_;
    std::array<double, 3> workspace_min_;
    std::array<double, 3> workspace_max_;
    std::array<double, 3> contact_position_{};

    double contact_on_;
    double contact_cancel_;
    double contact_off_;
    double contact_recover_;
    double debounce_;
    double release_delay_;
    double ramp_time_;

    double maximum_force_;
    double force_gain_;
    double admittance_;
    double stiffness_;
    double cartesian_damping_;
    double maximum_speed_;
    double maximum_distance_;
    double joint_margin_;

    double previous_time_ = 0;
    double phase_since_ = 0;
    double ramp_since_ = 0;
    double ramp_from_ = 0;
    double ramp_to_ = 0;
    double ramped_force_ = 0;
    bool gravity_only_;
    bool armed_ = true;
};
}  // namespace qhrr05
