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
    TaskController(const std::string& config_path, const std::string& shm_name);
    TaskController(const TaskController&) = delete;
    TaskController& operator=(const TaskController&) = delete;
    bool ReadInput();
    void Control();
    void WriteOutput();
    void WaitNextCycle();

private:
    shm_utils::Mapping shm_;
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
