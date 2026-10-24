#include "task_controller.hpp"
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TaskController::TaskController(const std::string& config_path, const std::string& shm_name)
    : shm_(shm_name), homing_(std::make_unique<HomingController>(config_path)),
      gravity_(std::make_unique<GravityCompensationController>(config_path)),
      magnetic_(std::make_unique<MagneticTouchController>(config_path)) {
    const double frequency =
        YAML::LoadFile(config_path)["task_controller"]["frequency_hz"].as<double>();

    if (!std::isfinite(frequency) || frequency <= 0)
        throw std::runtime_error("task_controller.frequency_hz must be positive and finite");

    const double period = 1e9 / frequency;

    if (period < 1 || period >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("task_controller.frequency_hz has an unrepresentable period");

    period_ns_ = static_cast<std::uint64_t>(period);
    started_ns_ = next_cycle_ns_ = shm_utils::MonotonicNs();
}

bool TaskController::ReadInput() {
    ShmState state;
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);

        if (shm.failed)
            throw std::runtime_error("SHM stopped after a process failure");

        if (shm.stopping)
            return false;

        state = shm.states[shm.state_sequence % 2];
        const auto& request = shm.requests[shm.request_sequence % 2];
        requested_mode_ = request.mode;
        request_sequence_ = request.motion_sequence;
        target_force_ = request.target_force;
    }

    if (requested_mode_ < 0 || requested_mode_ > 2)
        throw std::runtime_error("invalid controller mode");

    for (int joint = 0; joint < 3; ++joint)
        if (!std::isfinite(state.position[joint]) || !std::isfinite(state.velocity[joint]) ||
            !std::isfinite(state.gravity_torque[joint]))
            throw std::runtime_error("non-finite joint input");

    feedback_ = state;
    state_sequence_ = state.sequence;
    state_published_ns_ = state.published_ns;
    joint_position_ = state.position;
    joint_velocity_ = state.velocity;
    gravity_torque_ = state.gravity_torque;

    return true;
}

void TaskController::Control() {
    control_time_ = (shm_utils::MonotonicNs() - started_ns_) / 1e9;

    if (requested_mode_ != mode_ || request_sequence_ != active_request_sequence_) {
        if (requested_mode_ == 0)
            homing_->Start(control_time_, joint_position_.data());
        else if (requested_mode_ == 1)
            gravity_->Start(joint_position_.data());
        else
            magnetic_->Start(control_time_, feedback_);
        mode_ = requested_mode_;
        active_request_sequence_ = request_sequence_;
    }

    holding_ = false;

    if (mode_ == 0) {
        homing_->Compute(control_time_,
            joint_position_.data(),
            joint_velocity_.data(),
            torque_command_.data(),
            position_reference_.data());
        holding_ = homing_->Reference(control_time_).holding;
    } else if (mode_ == 1) {
        gravity_->Compute(joint_position_.data(),
            joint_velocity_.data(),
            gravity_torque_.data(),
            torque_command_.data(),
            position_reference_.data());
    } else {
        magnetic_->Compute(
            control_time_, feedback_, target_force_, torque_command_, position_reference_);
    }
}

void TaskController::WriteOutput() {
    auto& shm = *shm_.get();
    shm_utils::Lock lock(shm);

    if (shm.stopping)
        return;

    ++cycle_;
    auto& command = shm.commands[cycle_ % 2];

    command.sequence = cycle_;
    command.state_sequence = state_sequence_;
    command.state_published_ns = state_published_ns_;
    command.request_sequence = active_request_sequence_;

    command.control_time = control_time_;
    command.mode = mode_;
    command.holding = holding_;

    command.torque = torque_command_;
    command.reference = position_reference_;
    command.magnetic = mode_ == 2 ? magnetic_->telemetry() : MagneticTelemetry{};

    command.published_ns = shm_utils::MonotonicNs();

    shm.command_sequence = cycle_;
    shm_utils::Notify(shm);
}

void TaskController::WaitNextCycle() {
    next_cycle_ns_ += period_ns_;
    auto& shm = *shm_.get();
    shm_utils::Lock lock(shm);

    while (!shm.stopping && shm_utils::MonotonicNs() < next_cycle_ns_)
        lock.WaitUntil(next_cycle_ns_);
}
}  // namespace qhrr05
