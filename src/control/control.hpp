#pragma once

#include "motion_controllers/homing_controller.hpp"
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/magnetic_touch_controller.hpp"
#include "motion_controllers/tracking_controller.hpp"
#include "motion_controllers/ball_following_controller.hpp"
#include "motion_controllers/zero_set_controller.hpp"
#include "motion_controllers/damping_controller.hpp"
#include "motion_controllers/two_order_anticogging.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "ipc/shm_utils.hpp"
#include "interfaces/input/interface.hpp"
#include "interfaces/observations.hpp"
#include "safety/safety_layer.hpp"

namespace qhrr05 {
// Shared configuration only: no controller, model, transport or runtime state.
inline bool ReadTorqueRefAsPosition(const YAML::Node& config) {
    const auto value = config["torque_reference_mode"];
    if (!value || !value.IsScalar())
        throw std::runtime_error(
            "torque_reference_mode must be torque_ref_as_position, native_torque_ref or rmd_torque_ref");
    const auto mode = value.as<std::string>();
    if (mode != "torque_ref_as_position" && mode != "native_torque_ref" && mode != "rmd_torque_ref")
        throw std::runtime_error(
            "torque_reference_mode must be torque_ref_as_position, native_torque_ref or rmd_torque_ref");
    return mode == "torque_ref_as_position";
}

inline std::array<double, 3> ReadControlTorqueLimits(const YAML::Node& config) {
    for (const char* name : {"homing",
             "gravity_compensation",
             "magnetic_touch",
             "tracking",
             "ball_following",
             "zero_set"}) {
        const auto section = config[name];
        if (section && (section["torque_limit_nm"] || section["output_torque_limit_nm"]))
            throw std::runtime_error(
                std::string(name) + ": move output torque limits to limits.torque_nm");
    }
    if (!config["limits"] || !config["limits"]["torque_nm"])
        throw std::runtime_error("limits.torque_nm is required for every controller");
    const auto value = config["limits"]["torque_nm"];
    std::array<double, 3> limits;
    if (value.IsScalar())
        limits.fill(value.as<double>());
    else {
        const auto values = value.as<std::vector<double>>();
        if (values.size() != 3)
            throw std::runtime_error("limits.torque_nm requires a scalar or three joint values");
        std::copy(values.begin(), values.end(), limits.begin());
    }
    for (double limit : limits)
        if (!std::isfinite(limit) || limit <= 0)
            throw std::runtime_error("limits.torque_nm must be finite and positive");
    return limits;
}

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
    void ControlZeroSet();
    bool ZeroComplete() const;
    shm_utils::Mapping shm_;
    InputInterface input_;
    SafetyLayer safety_;
    InputContext input_context_{};
    JointObservation observation_{};
    DriveState drive_state_ = DriveState::Disarmed;
    std::uint64_t drive_request_sequence_ = 0, clear_fault_sequence_ = 0;
    bool clearing_transport_fault_ = false;
    double armed_at_ = 0, startup_timeout_ = 2;
    std::string fault_reason_, last_input_rejection_;
    void Fault(const std::string& reason);
    std::string config_path_;
    bool zero_required_ = false, zero_ready_ = false;
    bool zero_started_ = false;
    double zero_clock_ = 0, zero_last_control_time_ = 0;
    std::array<bool, 3> zero_issued_{};
    std::array<std::uint64_t, 3> zero_requests_{}, zero_completed_{}, zero_resume_{};
    std::array<double, 3> zero_positions_{};
    bool can_transport_ = true;
    ZeroSetTelemetry zero_status_{};
    std::array<double, 3> encoder_position_{};

    // Current joint inputs, copied once from the locked motor domain.
    std::array<double, 3> joint_position_{}, joint_velocity_{}, joint_acceleration_{};
    std::array<double, 3> motor_torque_{};

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
    std::array<bool, 3> acceleration_valid_{};

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
    std::unique_ptr<DampingController> damping_;
    std::unique_ptr<TwoOrderAnticogging> anticogging_;
    std::unique_ptr<ZeroSetController> zero_set_;

    std::array<double, 3> position_reference_{}, torque_command_{};
    std::array<double, 3> anticogging_output_nm_{}, anticogging_clip_until_{};
    std::array<unsigned, 3> anticogging_clipping_{};
    bool torque_ref_as_position_ = true;
    bool rmd_torque_ref_ = false;
    std::uint16_t rmd_damping_constant_ = 5;
    std::array<double, 3> torque_limits_{};
    std::array<MitCommand, 3> motor_targets_{};
    bool motor_commands_enabled_ = false;
    std::array<bool, 3> enabled_axes_{};
};
}  // namespace qhrr05
