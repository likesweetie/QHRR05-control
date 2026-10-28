#pragma once

#include <array>
#include <string>
#include <memory>
#include <vector>
#include <mujoco/mujoco.h>
#include "ipc/control_types.hpp"
#include "motion_controllers/tracking_ilc.hpp"

namespace qhrr05 {
class TrackingController {
public:
    explicit TrackingController(const std::string& config_path);

    void Start(double time, const std::array<double, 3>& toe_position,
        const std::array<double, 3>& joint_position,
        TrackingPlane plane = TrackingPlane::XY);
    void Compute(double time,
        const std::array<double, 3>& joint_position,
        const std::array<double, 3>& joint_velocity,
        const std::array<double, 3>& gravity_torque,
        const std::array<double, 9>& jacobian,
        const std::array<double, 3>& toe_position,
        std::array<double, 3>& torque);

    const std::array<double, 3>& kp_motor() const {
        return kp_motor_;
    }

    const std::array<double, 3>& kd_motor() const {
        return kd_motor_;
    }

    const std::array<double, 3>& toe_reference() const {
        return toe_reference_;
    }

    const std::array<double, 3>& toe_velocity_reference() const {
        return toe_velocity_reference_;
    }

    const std::array<double, 3>& toe_acceleration_reference() const {
        return toe_acceleration_reference_;
    }

    const std::array<double, 3>& joint_reference() const {
        return joint_reference_;
    }

    const std::array<double, 3>& joint_velocity_reference() const {
        return joint_velocity_reference_;
    }

    const std::array<double, 3>& joint_acceleration_reference() const {
        return joint_acceleration_reference_;
    }

    const std::array<double, 3>& inverse_dynamics_torque() const {
        return inverse_dynamics_torque_;
    }

    bool inverse_dynamics_enabled() const {
        return method_ == TrackingMethod::InverseDynamics;
    }

    bool joint_ik_enabled() const {
        return method_ != TrackingMethod::Impedance || ilc_.enabled();
    }

    TrackingMethod method() const {
        return method_;
    }

    const TrackingILC& ilc() const {
        return ilc_;
    }

    const std::array<double, 3>& ctc_torque() const {
        return ctc_torque_;
    }

    double ik_error() const {
        return ik_error_;
    }

    double minimum_singular_value() const {
        return minimum_singular_value_;
    }

    double jacobian_damping() const {
        return jacobian_damping_;
    }

    bool ik_failed() const {
        return ik_failed_;
    }

    bool checking() const {
        return checking_;
    }

    double check_progress() const {
        return double(checked_samples_) / (trajectory_samples_ + 1);
    }

    bool torque_saturated() const {
        return torque_saturated_;
    }

    TrackingPlane plane() const {
        return plane_;
    }

private:
    TrackingILC ilc_;
    void BuildArcLengthTables();
    void BuildAccelerationProfiles();
    double PhaseTimeDensity(TrackingPlane plane, double phase) const;
    void ComputeReference(double elapsed);
    bool CheckTrajectory();
    bool CheckCondition(const std::array<double, 9>& jacobian);
    bool SolvePosition(std::array<double, 9>& jacobian);
    void Hold(const std::array<double, 3>& joint_position);
    bool ComputeJointReference();
    bool ComputeDynamicsTorque(const std::array<double, 3>& joint_position,
        const std::array<double, 3>& joint_velocity);
    bool SolveJacobian(const std::array<double, 9>& jacobian,
        const std::array<double, 3>& cartesian, std::array<double, 3>& result);

    std::array<double, 3> kp_motor_, kd_motor_, stiffness_, damping_, torque_limit_;
    std::array<double, 3> center_{}, toe_reference_{}, toe_velocity_reference_{};
    std::array<double, 3> toe_acceleration_reference_{};
    std::array<double, 3> joint_reference_{}, joint_velocity_reference_{};
    std::array<double, 3> joint_acceleration_reference_{}, inverse_dynamics_torque_{};
    std::array<double, 3> ctc_kp_{}, ctc_kd_{}, ctc_torque_{};
    TrackingMethod method_ = TrackingMethod::InverseDynamics;
    std::array<double, 3> amplitude_;
    std::array<std::vector<double>, 3> arc_lengths_;
    // Smooth periodic log phase speed; initialized once, never optimized in Control().
    static constexpr int kTimingHarmonics = 24;
    std::array<std::array<double, kTimingHarmonics>, 3> timing_coefficients_{};
    std::array<std::vector<double>, 3> phase_times_;
    bool uniform_acceleration_ = false;
    int arc_length_samples_;
    int trajectory_samples_ = 0;
    double condition_limit_ = 0, condition_number_ = 0;
    int checked_samples_ = 0;
    bool checking_ = false;
    double peak_path_condition_ = 0;
    std::array<double, 3> validation_joint_{};
    TrackingPlane plane_ = TrackingPlane::XY;
    double period_, ramp_time_, started_ = 0;
    bool torque_saturated_ = false;
    bool ik_failed_ = false;
    const char* ik_failure_reason_ = "";

    // Controller-local mathematical model; never shares the simulator's mjData or clock.
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> id_model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> id_data_{nullptr, mj_deleteData};
    double id_alpha_ = 0, regularization_ = 0, ik_tolerance_ = 0, ik_step_limit_ = 0, ik_error_ = 0;
    double maximum_regularization_ = 0, singularity_threshold_ = 0;
    double minimum_singular_value_ = 0, jacobian_damping_ = 0;
    int ik_iterations_ = 0, toe_site_ = -1;
};
}  // namespace qhrr05
