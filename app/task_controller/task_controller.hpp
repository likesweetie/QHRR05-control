#pragma once

#include "motion_controllers/homing_controller.hpp"
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/magnetic_touch_controller.hpp"
#include <array>
#include <memory>
#include <string>
#include "shm_utils.hpp"

namespace qhrr05 {
class TaskController {
public:
    TaskController(const std::string& config_path,
        const std::string& shm_name,
        const std::string& motor_config_path,
        bool without_can = false);
    TaskController(const TaskController&) = delete;
    TaskController& operator=(const TaskController&) = delete;
    bool ReadInput();
    void Control();
    void WriteOutput();
    void WaitNextCycle();
    void Release();

private:
    shm_utils::Mapping shm_;
    std::unique_ptr<shm_utils::Mapping> can_;
    ShmCanState can_feedback_{};
    std::array<double, 3> encoder_origin_{};
    std::array<std::uint64_t, 3> received_sequence_{}, received_ns_{};
    std::array<double, 3> measured_velocity_{}, measured_acceleration_{};
    std::uint64_t measurement_sequence_ = 0, measurement_ns_ = 0;
    std::array<openrobot::PositionTarget, 3> motor_targets_{};
    std::uint64_t cycle_ = 0, state_sequence_ = 0, state_published_ns_ = 0;
    std::uint64_t request_sequence_ = 0, active_request_sequence_ = 0;
    std::uint64_t started_ns_ = 0, next_cycle_ns_ = 0, period_ns_ = 0;
    std::unique_ptr<HomingController> homing_;
    std::unique_ptr<GravityCompensationController> gravity_;
    std::unique_ptr<MagneticTouchController> magnetic_;
    ShmState feedback_{};
    double target_force_ = 3.0;
    int mode_ = -1;
    int requested_mode_ = 0;
    double control_time_ = 0;
    bool holding_ = false;
    std::array<double, 3> joint_position_{};  // rad
    std::array<double, 3> joint_velocity_{};  // rad/s
    std::array<double, 3> gravity_torque_{};  // N*m
    std::array<double, 3> torque_command_{};  // N*m
    std::array<double, 3> position_reference_{};  // rad
};
}  // namespace qhrr05
