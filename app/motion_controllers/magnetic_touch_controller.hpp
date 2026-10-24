#pragma once
#include "reaction_force_observer.hpp"
#include <string>

namespace qhrr05 {
class MagneticTouchController {
public:
    explicit MagneticTouchController(const std::string& config_path);
    void Start(double time, const ShmState& feedback);
    void Compute(double time, const ShmState& feedback, double target_force,
                 std::array<double, 3>& torque, std::array<double, 3>& reference);
    const MagneticTelemetry& telemetry() const { return telemetry_; }
private:
    void Transition(MagneticPhase phase, MagneticReason reason, double time);
    void Release(MagneticReason reason, double time);
    ReactionForceObserver observer_;
    MagneticTelemetry telemetry_{};
    std::array<double, 3> damping_, torque_limit_, workspace_min_, workspace_max_, contact_position_{};
    double contact_on_, contact_cancel_, contact_off_, contact_recover_, debounce_, release_delay_, ramp_time_;
    double maximum_force_, force_gain_, admittance_, stiffness_, cartesian_damping_, maximum_speed_, maximum_distance_, joint_margin_;
    double previous_time_ = 0, phase_since_ = 0, ramp_since_ = 0, ramp_from_ = 0, ramp_to_ = 0, ramped_force_ = 0;
    bool armed_ = true;
};
}  // namespace qhrr05
