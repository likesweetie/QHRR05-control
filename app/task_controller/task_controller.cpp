#include "task_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TaskController::TaskController(const std::string& config_path,
    const std::string& shm_name,
    const std::string& motor_config_path,
    bool without_can)
    : shm_(shm_name), homing_(std::make_unique<HomingController>(config_path)),
      gravity_(std::make_unique<GravityCompensationController>(config_path)),
      magnetic_(std::make_unique<MagneticTouchController>(config_path)) {
    // Simulation bootstrap only. Hardware stopper calibration replaces this offset later.
    encoder_origin_ = homing_->initial_position();

    if (!without_can) {
        const auto config = YAML::LoadFile(motor_config_path);
        can_ = std::make_unique<shm_utils::Mapping>(config["shm_name"].as<std::string>());
        auto& shm = *can_->get();
        shm_utils::Lock lock(shm);

        if (shm.failed || shm.stopping || !shm.can_states[shm.can_state_sequence % 2].running)
            throw std::runtime_error("CAN HAL is not initialized");
    }

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

    if (can_) {
        {
            auto& shm = *can_->get();
            shm_utils::Lock lock(shm);

            if (shm.failed)
                throw std::runtime_error("CAN HAL failed");

            if (shm.stopping)
                return false;

            can_feedback_ = shm.can_states[shm.can_state_sequence % 2];
        }

        if (!can_feedback_.running)
            return false;

        bool fresh = false;
        std::uint64_t newest_ns = measurement_ns_;

        for (int i = 0; i < 3; ++i) {
            const auto& motor = can_feedback_.motors[i];
            state.position[i] = encoder_origin_[i] + motor.feedback.position;
            state.velocity[i] = motor.feedback.velocity;
            state.motor_torque[i] = motor.feedback.current * can_feedback_.profile.kt_output;

            if (motor.feedback_sequence != received_sequence_[i]) {
                if (received_ns_[i] && motor.received_ns > received_ns_[i])
                    measured_acceleration_[i] = (motor.feedback.velocity - measured_velocity_[i]) /
                                                ((motor.received_ns - received_ns_[i]) / 1e9);

                received_sequence_[i] = motor.feedback_sequence;
                received_ns_[i] = motor.received_ns;
                measured_velocity_[i] = motor.feedback.velocity;
                newest_ns = std::max(newest_ns, motor.received_ns);
                fresh = true;
            }
        }

        if (fresh) {
            if (measurement_ns_ && newest_ns > measurement_ns_)
                state.measurement_dt = (newest_ns - measurement_ns_) / 1e9;

            measurement_ns_ = newest_ns;
            ++measurement_sequence_;
        }

        state.acceleration = measured_acceleration_;
        state.measurement_sequence = measurement_sequence_;
    }

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

    if (can_) {
        const auto& kp = mode_ == 0 ? homing_->kp()
                                    : (mode_ == 1 ? gravity_->kp_motor() : magnetic_->kp_motor());
        const auto& kd = mode_ == 0 ? homing_->kd()
                                    : (mode_ == 1 ? gravity_->kd_motor() : magnetic_->kd_motor());
        const auto home = homing_->Reference(control_time_);
        bool profiles_ready = true;

        for (const auto& motor : can_feedback_.motors)
            profiles_ready &= motor.profile_verified;

        for (int i = 0; i < 3; ++i) {
            // Remove the motor D term already included in torque_command_, then
            // represent the resulting effort as an actual MIT position target.
            const double position =
                mode_ == 0 ? home.position[i]
                           : joint_position_[i] +
                                 (torque_command_[i] + kd[i] * joint_velocity_[i]) / kp[i];
            motor_targets_[i] = {profiles_ready,
                position - encoder_origin_[i],
                mode_ == 0 ? home.velocity[i] : 0,
                kp[i],
                kd[i]};
            openrobot::EncodePosition(motor_targets_[i], can_feedback_.profile);
        }
    }
}

void TaskController::WriteOutput() {
    {
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

    if (can_) {
        auto& shm = *can_->get();
        shm_utils::Lock lock(shm);

        if (shm.failed)
            throw std::runtime_error("CAN HAL failed during output");

        if (shm.stopping)
            return;

        const auto sequence = shm.motor_target_sequence + 1;
        auto& target = shm.motor_targets[sequence % 2];
        target.sequence = sequence;
        target.published_ns = shm_utils::MonotonicNs();
        target.motors = motor_targets_;
        shm.motor_target_sequence = sequence;
        shm_utils::Notify(shm);
    }
}

void TaskController::Release() {
    if (!can_)
        return;

    auto& shm = *can_->get();
    shm_utils::Lock lock(shm);

    if (shm.stopping)
        return;

    const auto sequence = shm.motor_target_sequence + 1;
    shm.motor_targets[sequence % 2] = {sequence, shm_utils::MonotonicNs(), {}};
    shm.motor_target_sequence = sequence;
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
