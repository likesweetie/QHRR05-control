#pragma once

#include "motion_controllers/homing_controller.hpp"
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/magnetic_touch_controller.hpp"
#include "motion_controllers/tracking_controller.hpp"
#include "motion_controllers/ball_following_controller.hpp"
#include <array>
#include <memory>
#include <string>
#include "ipc/shm_utils.hpp"
#include "interfaces/input/interface.hpp"
#include "ipc/simulation/channel.hpp"
#include "safety/safety_layer.hpp"

namespace qhrr05 {
class TaskController {
public:
    TaskController(
        const std::string& config_path, const std::string& shm_name, bool without_can = false);
    TaskController(const TaskController&) = delete;
    TaskController& operator=(const TaskController&) = delete;
    bool ReadInput();
    void Control();
    void WriteOutput();
    void WaitNextCycle();
    void Release();

private:
    shm_utils::Mapping shm_;
    InputInterface input_;
    SafetyLayer safety_;
    simulation_ipc::Client simulation_channel_;
    SimulationApproval simulation_approval_{};
    std::string last_simulation_error_;
    std::string last_input_rejection_;
    bool without_can_ = false;

    // Current joint inputs, copied once from the locked motor domain.
    std::array<double, 3> joint_position_{}, joint_velocity_{}, joint_acceleration_{};
    std::array<double, 3> motor_torque_{}, encoder_origin_{};

    // Current backend outputs, evaluated from the measured encoder state outside SHM locks.
    std::array<double, 3> gravity_torque_{}, bias_torque_{}, passive_torque_{}, friction_limit_{};
    std::array<double, 9> mass_matrix_{}, jacobian_{};
    std::array<double, 3> toe_position_{}, joint_min_{}, joint_max_{};
    std::uint64_t model_sequence_ = 0, model_published_ns_ = 0;
    std::uint64_t measurement_sequence_ = 0, measurement_ns_ = 0;
    double measurement_dt_ = .001;

    // Previous samples are retained only for acceleration and fresh-sample tracking.
    std::array<std::uint64_t, 3> previous_received_sequence_{}, previous_received_ns_{};
    std::array<double, 3> previous_velocity_{};

    double target_force_ = 3.0;
    int mode_ = -1, requested_mode_ = 5;
    TrackingPlane requested_tracking_plane_ = TrackingPlane::XY;
    std::uint64_t request_sequence_ = 0, active_request_sequence_ = 0;
    std::uint64_t started_ns_ = 0, next_cycle_ns_ = 0, period_ns_ = 0;
    double control_time_ = 0;
    bool holding_ = false;
    std::array<double, 3> damping_toe_reference_{};
    bool damping_reference_valid_ = false;
    std::array<double, 3> joystick_direction_{}, requested_ball_position_{};
    double joystick_l2_ = 0, joystick_r2_ = 0;
    bool joystick_connected_ = false;
    std::uint64_t ball_target_sequence_ = 0, active_ball_target_sequence_ = 0;

    std::unique_ptr<ModelBackend> model_backend_;
    std::unique_ptr<HomingController> homing_;
    std::unique_ptr<GravityCompensationController> gravity_;
    std::unique_ptr<MagneticTouchController> magnetic_;
    std::unique_ptr<TrackingController> tracking_;
    std::unique_ptr<BallFollowingController> ball_;

    std::array<double, 3> torque_command_{}, position_reference_{};
    std::array<double, 3> torque_limits_{};
    std::array<MitCommand, 3> motor_targets_{};
    bool motor_commands_enabled_ = false;
};
}  // namespace qhrr05
