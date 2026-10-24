#pragma once

#include "reaction_force_observer.hpp"
#include <string>

namespace qhrr05 {
class MagneticTouchController {
public:
    explicit MagneticTouchController(const std::string& config_path);
    void Start(double time, const ShmState& feedback);
    void Compute(double time,
        const ShmState& feedback,
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
    bool armed_ = true;
};
}  // namespace qhrr05
