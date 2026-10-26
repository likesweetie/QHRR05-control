#include "task_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TaskController::TaskController(
    const std::string& config_path, const std::string& shm_name, bool without_can)
    : shm_(shm_name), without_can_(without_can),
      homing_(std::make_unique<HomingController>(config_path)),
      gravity_(std::make_unique<GravityCompensationController>(config_path)),
      magnetic_(std::make_unique<MagneticTouchController>(config_path)) {
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        const auto& motors = shm.motors[shm.motor_sequence % 2];

        if (shm.can_enabled == without_can_ ||
            (!without_can_ && (shm.failed || shm.stopping || !motors.running)))
            throw std::runtime_error("SHM transport does not match controller mode");
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
    MotionFeedback state;
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);

        if (shm.failed)
            throw std::runtime_error("SHM stopped after a process failure");

        if (shm.stopping)
            return false;

        static_cast<ModelData&>(state) = shm.model[shm.model_sequence % 2];
        model_sequence_ = shm.model_sequence;
        model_published_ns_ = state.published_ns;
        motor_data_ = shm.motors[shm.motor_sequence % 2];
        const auto& control = shm.control[shm.control_sequence % 2];
        requested_mode_ = control.requested_mode;
        request_sequence_ = control.motion_sequence;
        target_force_ = control.target_force;
    }

    if (requested_mode_ < 0 || requested_mode_ > 2)
        throw std::runtime_error("invalid controller mode");

    bool fresh = false;
    std::uint64_t newest_ns = measurement_ns_;

    for (int i = 0; i < 3; ++i) {
        encoder_origin_[i] = motor_data_.position_offset[i];
        state.position[i] = motor_data_.position_offset[i] + motor_data_.feedback[i].position;
        state.velocity[i] = motor_data_.feedback[i].velocity;
        state.motor_torque[i] = motor_data_.feedback[i].current * motor_data_.profile.kt_output;

        if (motor_data_.received_sequence[i] != received_sequence_[i]) {
            const auto samples = motor_data_.received_sequence[i] - received_sequence_[i];
            const double dt = without_can_ ? samples * motor_data_.sample_dt
                                           : (motor_data_.received_ns[i] - received_ns_[i]) / 1e9;

            if (received_sequence_[i] && dt > 0)
                measured_acceleration_[i] =
                    (motor_data_.feedback[i].velocity - measured_velocity_[i]) / dt;

            received_sequence_[i] = motor_data_.received_sequence[i];
            received_ns_[i] = motor_data_.received_ns[i];
            measured_velocity_[i] = motor_data_.feedback[i].velocity;
            newest_ns = std::max(newest_ns, motor_data_.received_ns[i]);
            fresh = true;
        }

        if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i]) ||
            !std::isfinite(state.gravity_torque[i]))
            throw std::runtime_error("non-finite joint input");
    }

    state.measurement_dt = motor_data_.sample_dt;

    if (fresh) {
        if (!without_can_ && measurement_ns_ && newest_ns > measurement_ns_)
            state.measurement_dt = (newest_ns - measurement_ns_) / 1e9;

        measurement_ns_ = newest_ns;

        if (without_can_)
            measurement_sequence_ =
                *std::max_element(received_sequence_.begin(), received_sequence_.end());
        else
            ++measurement_sequence_;
    }

    state.acceleration = measured_acceleration_;
    state.measurement_sequence = measurement_sequence_;
    feedback_ = state;
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

    if (!without_can_) {
        const auto& kp = mode_ == 0 ? homing_->kp()
                                    : (mode_ == 1 ? gravity_->kp_motor() : magnetic_->kp_motor());
        const auto& kd = mode_ == 0 ? homing_->kd()
                                    : (mode_ == 1 ? gravity_->kd_motor() : magnetic_->kd_motor());
        const auto home = homing_->Reference(control_time_);
        bool profiles_ready = true;

        for (bool verified : motor_data_.profile_verified)
            profiles_ready &= verified;

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
            openrobot::EncodePosition(motor_targets_[i], motor_data_.profile);
        }
    }
}

void TaskController::WriteOutput() {
    auto& shm = *shm_.get();
    shm_utils::Lock lock(shm);

    if (shm.failed)
        throw std::runtime_error("SHM failed during output");

    if (shm.stopping)
        return;

    auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);
    ++motors.command_sequence;
    motors.command_published_ns = shm_utils::MonotonicNs();

    for (int i = 0; i < 3; ++i) {
        motors.command[i] = motor_targets_[i];
        motors.debug_torque[i] = torque_command_[i];
        motors.reference[i] = position_reference_[i];
    }

    auto& control = shm_utils::Next(shm.control, shm.control_sequence);
    control.command_sequence = motors.command_sequence;
    control.published_ns = motors.command_published_ns;
    control.input_model_sequence = model_sequence_;
    control.input_model_published_ns = model_published_ns_;
    control.active_motion_sequence = active_request_sequence_;
    control.control_time = control_time_;
    control.mode = mode_;
    control.holding = holding_;
    control.magnetic = mode_ == 2 ? magnetic_->telemetry() : MagneticTelemetry{};

    ++shm.motor_sequence;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
}

void TaskController::Release() {
    if (without_can_)
        return;

    auto& shm = *shm_.get();
    shm_utils::Lock lock(shm);

    if (shm.stopping)
        return;

    auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);

    for (auto& target : motors.command)
        target.enabled = false;

    ++motors.command_sequence;
    motors.command_published_ns = shm_utils::MonotonicNs();
    ++shm.motor_sequence;
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
