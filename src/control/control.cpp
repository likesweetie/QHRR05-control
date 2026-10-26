#include "control/control.hpp"
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
        const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];

        if (shm.can_enabled == without_can_ ||
            (!without_can_ && (shm.failed || shm.stopping || !motors.running)))
            throw std::runtime_error("SHM transport does not match controller mode");
    }

    const double frequency = YAML::LoadFile(config_path)["control"]["frequency_hz"].as<double>();

    if (!std::isfinite(frequency) || frequency <= 0)
        throw std::runtime_error("control.frequency_hz must be positive and finite");

    const double period = 1e9 / frequency;

    if (period < 1 || period >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("control.frequency_hz has an unrepresentable period");

    period_ns_ = static_cast<std::uint64_t>(period);
    started_ns_ = next_cycle_ns_ = shm_utils::MonotonicNs();
}

bool TaskController::ReadInput() {
    std::array<std::uint64_t, 3> received_sequence, received_ns;
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);

        if (shm.failed)
            throw std::runtime_error("SHM stopped after a process failure");

        if (shm.stopping)
            return false;

        const auto& model = shm.model[shm.model_sequence % kShmBufferCount];
        gravity_torque_ = model.gravity_torque;
        bias_torque_ = model.bias_torque;
        passive_torque_ = model.passive_torque;
        friction_limit_ = model.friction_limit;
        mass_matrix_ = model.mass;
        jacobian_ = model.jacobian;
        toe_position_ = model.toe_position;
        joint_min_ = model.joint_min;
        joint_max_ = model.joint_max;
        model_sequence_ = shm.model_sequence;
        model_published_ns_ = model.published_ns;

        const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        motor_commands_enabled_ = !without_can_;

        for (int i = 0; i < 3; ++i) {
            encoder_origin_[i] = motors.position_offset[i];
            joint_position_[i] = motors.position_offset[i] + motors.feedback[i].position;
            joint_velocity_[i] = motors.feedback[i].velocity;
            motor_torque_[i] = motors.motor_torque[i];
            motor_commands_enabled_ &= motors.profile_verified[i];
        }

        received_sequence = motors.received_sequence;
        received_ns = motors.received_ns;
        measurement_dt_ = motors.sample_dt;

        const auto& control = shm.control[shm.control_sequence % kShmBufferCount];
        requested_mode_ = control.requested_mode;
        request_sequence_ = control.motion_sequence;
        target_force_ = control.target_force;
    }

    if (requested_mode_ < 0 || requested_mode_ > 2)
        throw std::runtime_error("invalid controller mode");

    bool fresh = false;
    std::uint64_t newest_ns = measurement_ns_;

    for (int i = 0; i < 3; ++i) {
        if (received_sequence[i] != previous_received_sequence_[i]) {
            const auto samples = received_sequence[i] - previous_received_sequence_[i];
            const double dt = without_can_ ? samples * measurement_dt_
                                           : (received_ns[i] - previous_received_ns_[i]) / 1e9;

            if (previous_received_sequence_[i] && dt > 0)
                joint_acceleration_[i] = (joint_velocity_[i] - previous_velocity_[i]) / dt;

            previous_received_sequence_[i] = received_sequence[i];
            previous_received_ns_[i] = received_ns[i];
            previous_velocity_[i] = joint_velocity_[i];
            newest_ns = std::max(newest_ns, received_ns[i]);
            fresh = true;
        }

        if (!std::isfinite(joint_position_[i]) || !std::isfinite(joint_velocity_[i]) ||
            !std::isfinite(gravity_torque_[i]))
            throw std::runtime_error("non-finite joint input");
    }

    if (fresh) {
        if (!without_can_ && measurement_ns_ && newest_ns > measurement_ns_)
            measurement_dt_ = (newest_ns - measurement_ns_) / 1e9;

        measurement_ns_ = newest_ns;

        if (without_can_)
            measurement_sequence_ = *std::max_element(
                previous_received_sequence_.begin(), previous_received_sequence_.end());
        else
            ++measurement_sequence_;
    }

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
            magnetic_->Start(control_time_, toe_position_);
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
            bias_torque_.data(),
            torque_command_.data(),
            position_reference_.data());
    } else {
        magnetic_->Compute(control_time_,
            joint_position_,
            joint_velocity_,
            joint_acceleration_,
            motor_torque_,
            gravity_torque_,
            bias_torque_,
            passive_torque_,
            friction_limit_,
            mass_matrix_,
            jacobian_,
            toe_position_,
            joint_min_,
            joint_max_,
            measurement_sequence_,
            measurement_dt_,
            target_force_,
            torque_command_,
            position_reference_);
    }

    if (!without_can_) {
        const auto& kp = mode_ == 0 ? homing_->kp()
                                    : (mode_ == 1 ? gravity_->kp_motor() : magnetic_->kp_motor());
        const auto& kd = mode_ == 0 ? homing_->kd()
                                    : (mode_ == 1 ? gravity_->kd_motor() : magnetic_->kd_motor());
        const auto home = homing_->Reference(control_time_);
        for (int i = 0; i < 3; ++i) {
            // Remove the motor D term already included in torque_command_, then
            // represent the resulting effort as an actual MIT position target.
            const double position =
                mode_ == 0 ? home.position[i]
                           : joint_position_[i] +
                                 (torque_command_[i] + kd[i] * joint_velocity_[i]) / kp[i];
            motor_targets_[i] = {
                position - encoder_origin_[i], mode_ == 0 ? home.velocity[i] : 0, kp[i], kd[i], 0};
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
        motors.command_enabled[i] = motor_commands_enabled_;
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

    motors.command_enabled.fill(false);

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
