#include "control/control.hpp"
#include "control/limits.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TaskController::TaskController(
    const std::string& config_path, const std::string& shm_name, bool without_can)
    : shm_(shm_name), input_(shm_name), safety_(config_path), config_path_(config_path),
      model_backend_(MakeMujocoBackend(config_path)),
      homing_(std::make_unique<HomingController>(config_path)),
      gravity_(std::make_unique<GravityCompensationController>(config_path)),
      magnetic_(std::make_unique<MagneticTouchController>(config_path)),
      tracking_(std::make_unique<TrackingController>(config_path, *model_backend_)),
      ball_(std::make_unique<BallFollowingController>(config_path, *model_backend_)) {
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];

        if (shm.can_enabled == without_can ||
            (!without_can && (shm.failed || shm.stopping || !motors.running)))
            throw std::runtime_error("SHM transport does not match controller mode");
        zero_status_ = shm.control[shm.control_sequence % kShmBufferCount].zero_set;
    }

    const auto config = YAML::LoadFile(config_path);
    torque_limits_ = ReadControlTorqueLimits(config);
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        auto& state = shm_utils::Next(shm.control, shm.control_sequence);
        if (!state.command_sequence)
            state.target_force = config["magnetic_touch"]["target_force_n"].as<double>();
        ++shm.control_sequence;
    }
    startup_timeout_ = config["control"]["startup_timeout_sec"].as<double>();
    if (!std::isfinite(startup_timeout_) || startup_timeout_ <= 0)
        throw std::runtime_error("control.startup_timeout_sec must be positive and finite");
    const double frequency = config["control"]["frequency_hz"].as<double>();

    if (!std::isfinite(frequency) || frequency <= 0)
        throw std::runtime_error("control.frequency_hz must be positive and finite");

    const double period = 1e9 / frequency;

    if (period < 1 || period >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("control.frequency_hz has an unrepresentable period");

    period_ns_ = static_cast<std::uint64_t>(period);
    started_ns_ = next_cycle_ns_ = shm_utils::MonotonicNs();
}

void TaskController::Fault(const std::string& reason) {
    if (drive_state_ != DriveState::Fault || reason != fault_reason_)
        std::cerr << "[control] FAULT: " << reason << "; outputs released; clear-fault required\n";
    drive_state_ = DriveState::Fault;
    fault_reason_ = reason;
    motor_commands_enabled_ = false;
    enabled_axes_.fill(false);
}

bool TaskController::ReadInput() {
    input_context_ = input_.Poll();
    const auto command = input_.Receive();
    std::string rejection;
    std::uint64_t approved_sequence = 0;
    MotorData motors;
    ControlData candidate;
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        if (shm.failed)
            throw std::runtime_error("SHM stopped after a process failure");
        if (shm.stopping)
            return false;
        motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        candidate = shm.control[shm.control_sequence % kShmBufferCount];
    }
    observation_ = ReadObservation(motors);
    zero_required_ = !CoordinatesValid(observation_.coordinate_valid);
    zero_ready_ = observation_.position_velocity && observation_.torque_valid && observation_.ready;
    if (!clearing_transport_fault_ && motors.transport_fault != TransportFault::None)
        Fault(TransportFaultName(motors.transport_fault));
    if (motors.transport_fault == TransportFault::None)
        clearing_transport_fault_ = false;
    if (input_context_.start_calibration && zero_required_) {
        candidate.requested_mode = 6;
        ++candidate.motion_sequence;
    }
    if (command) {
        if (command->set_mode) {
            candidate.requested_mode = command->mode;
            candidate.requested_tracking_plane = command->plane;
            ++candidate.motion_sequence;
        } else if (command->drive_action != DriveAction::None) {
            candidate.requested_drive_action = command->drive_action;
            candidate.requested_motor_axis = command->motor_axis;
            ++candidate.drive_request_sequence;
        } else if (!command->auxiliary)
            candidate.target_force = command->force;
    }
    try {
        const bool priority_action =
            candidate.drive_request_sequence != drive_request_sequence_ &&
            (candidate.requested_drive_action == DriveAction::Release ||
                candidate.requested_drive_action == DriveAction::DisableMotor ||
                candidate.requested_drive_action == DriveAction::ClearFault);
        if (candidate.requested_drive_action == DriveAction::DisableMotor &&
            (candidate.requested_motor_axis < -1 || candidate.requested_motor_axis >= 3))
            throw std::runtime_error("invalid motor axis");
        if (!priority_action)
            safety_.Validate(candidate, false, false);
        if (!priority_action && candidate.requested_drive_action != DriveAction::EnableMotor &&
            candidate.requested_mode != 5 && candidate.requested_mode != 6 && zero_required_)
            throw std::runtime_error("joint coordinates are not calibrated; select Zero set first");
        if (candidate.drive_request_sequence != drive_request_sequence_) {
            const auto action = candidate.requested_drive_action;
            if (action == DriveAction::Arm) {
                if (drive_state_ == DriveState::Active)
                    throw std::runtime_error(
                        "drive is already ACTIVE; release before arming again");
                if (drive_state_ == DriveState::Fault ||
                    motors.transport_fault != TransportFault::None)
                    throw std::runtime_error("clear the drive fault before arming");
                auto needed =
                    candidate.requested_mode == 5 ? JointVelocity : EncoderPosition | JointVelocity;
                if (candidate.requested_mode == 6 || candidate.requested_mode == 2)
                    needed |= ConvertedTorque;
                for (auto available : motors.supported)
                    if ((available & needed) != needed)
                        throw std::runtime_error(
                            "required observation is not supported by the device");
                drive_state_ = DriveState::Active;
                enabled_axes_.fill(true);
                armed_at_ = (shm_utils::MonotonicNs() - started_ns_) / 1e9;
                // Re-enter the selected motion at the measured position on a new arm.
                mode_ = -1;
                fault_reason_.clear();
                std::cout << "[control] ACTIVE requested; waiting for valid feedback\n";
            } else if (action == DriveAction::Release) {
                enabled_axes_.fill(false);
                if (drive_state_ != DriveState::Fault)
                    drive_state_ = DriveState::Disarmed;
                std::cout << "[control] DISARM requested\n";
            } else if (action == DriveAction::ClearFault) {
                if (drive_state_ == DriveState::Active)
                    throw std::runtime_error("release outputs before clearing a fault");
                ++clear_fault_sequence_;
                clearing_transport_fault_ = true;
                drive_state_ = DriveState::Disarmed;
                enabled_axes_.fill(false);
                fault_reason_.clear();
                std::cout << "[control] fault clear requested; remains DISARMED\n";
            } else if (action == DriveAction::EnableMotor || action == DriveAction::DisableMotor) {
                if (drive_state_ == DriveState::Fault ||
                    motors.transport_fault != TransportFault::None)
                    throw std::runtime_error("clear the drive fault before changing motor enable");
                if (action == DriveAction::EnableMotor && drive_state_ == DriveState::Active)
                    throw std::runtime_error("release motion before enabling an individual motor");
                if (candidate.requested_motor_axis == -1)
                    enabled_axes_.fill(action == DriveAction::EnableMotor);
                else
                    enabled_axes_[candidate.requested_motor_axis] = action == DriveAction::EnableMotor;
                drive_state_ = std::any_of(enabled_axes_.begin(),
                                   enabled_axes_.end(),
                                   [](bool v) {
                                       return v;
                                   })
                                   ? DriveState::Armed
                                   : DriveState::Disarmed;
                mode_ = -1;
                std::cout
                    << "[control] individual motor enable changed; neutral output until Arm\n";
            }
            drive_request_sequence_ = candidate.drive_request_sequence;
        }
        if (command && command->auxiliary)
            approved_sequence = input_.ApproveAuxiliary();
        requested_mode_ = candidate.requested_mode;
        requested_tracking_plane_ = candidate.requested_tracking_plane;
        request_sequence_ = candidate.motion_sequence;
        target_force_ = candidate.target_force;
        joystick_direction_ = candidate.joystick_axes;
        joystick_l2_ = candidate.joystick_l2;
        joystick_r2_ = candidate.joystick_r2;
        joystick_connected_ = candidate.joystick_connected;
        requested_ball_position_ = candidate.requested_ball_position;
        ball_target_sequence_ = candidate.ball_target_sequence;
        if (!command || !command->auxiliary)
            approved_sequence = command && command->drive_action != DriveAction::None
                                    ? drive_request_sequence_
                                    : request_sequence_;
    } catch (const std::runtime_error& error) {
        rejection = error.what();
    }
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        auto& approved = shm_utils::Next(shm.control, shm.control_sequence);
        approved.requested_mode = requested_mode_;
        approved.requested_tracking_plane = requested_tracking_plane_;
        approved.motion_sequence = request_sequence_;
        approved.target_force = target_force_;
        approved.drive_request_sequence = drive_request_sequence_;
        approved.requested_drive_action = DriveAction::None;
        ++shm.control_sequence;
    }
    if (command)
        input_.Reply(*command, rejection, approved_sequence);
    if (!rejection.empty() && rejection != last_input_rejection_)
        std::cerr << "[safety] " << rejection << "; request discarded\n";
    last_input_rejection_ = rejection;

    joint_position_ = observation_.position;
    encoder_position_ = observation_.encoder;
    joint_velocity_ = observation_.velocity;
    motor_torque_ = observation_.torque;
    encoder_origin_ = motors.position_offset;
    bool fresh = false;
    for (int i = 0; i < 3; ++i) {
        if (observation_.sequence[i] != previous_received_sequence_[i]) {
            const double dt = observation_.sample_ns[i] > previous_received_ns_[i]
                                  ? (observation_.sample_ns[i] - previous_received_ns_[i]) / 1e9
                                  : 0;
            acceleration_valid_[i] =
                previous_received_sequence_[i] && dt > 0 && observation_.position_velocity;
            if (acceleration_valid_[i])
                joint_acceleration_[i] = (joint_velocity_[i] - previous_velocity_[i]) / dt;
            previous_received_sequence_[i] = observation_.sequence[i];
            previous_received_ns_[i] = observation_.sample_ns[i];
            previous_velocity_[i] = joint_velocity_[i];
            if (dt > 0)
                measurement_dt_ = dt;
            fresh = true;
        }
    }
    if (fresh)
        ++measurement_sequence_;
    motor_commands_enabled_ =
        (drive_state_ == DriveState::Active || drive_state_ == DriveState::Armed) &&
        std::all_of(motors.profile_verified.begin(), motors.profile_verified.end(), [](bool v) {
            return v;
        });
    if (observation_.position_velocity && !zero_required_ && (fresh || !model_published_ns_)) {
        const auto& state = model_backend_->Update(joint_position_, joint_velocity_);
        gravity_torque_ = state.gravity;
        bias_torque_ = state.bias;
        passive_torque_ = state.passive;
        friction_limit_ = state.friction;
        mass_matrix_ = state.mass;
        jacobian_ = state.jacobian;
        toe_position_ = state.toe_position;
        joint_min_ = state.joint_min;
        joint_max_ = state.joint_max;
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        auto& model = shm_utils::Next(shm.model, shm.model_sequence);
        model.published_ns = shm_utils::MonotonicNs();
        model.input_measurement_sequence = measurement_sequence_;
        model.valid = true;
        model.acceleration_valid = CoordinatesValid(acceleration_valid_);
        model.joint_acceleration = joint_acceleration_;
        model.gravity_torque = gravity_torque_;
        model.bias_torque = bias_torque_;
        model.passive_torque = passive_torque_;
        model.friction_limit = friction_limit_;
        model.mass = mass_matrix_;
        model.jacobian = jacobian_;
        model.toe_position = toe_position_;
        model.joint_min = joint_min_;
        model.joint_max = joint_max_;
        model_published_ns_ = model.published_ns;
        model_sequence_ = ++shm.model_sequence;
    } else if ((!observation_.position_velocity || zero_required_) && model_published_ns_) {
        model_published_ns_ = 0;
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        auto& model = shm_utils::Next(shm.model, shm.model_sequence);
        model.valid = false;
        ++shm.model_sequence;
    }
    return true;
}

void TaskController::Control() {
    control_time_ = (shm_utils::MonotonicNs() - started_ns_) / 1e9;
    const bool valid =
        (requested_mode_ == 5 ? observation_.velocity_valid : observation_.position_velocity) &&
        observation_.ready && (requested_mode_ == 5 || requested_mode_ == 6 || !zero_required_) &&
        (requested_mode_ != 2 || CoordinatesValid(acceleration_valid_)) &&
        ((requested_mode_ != 6 && requested_mode_ != 2) || observation_.torque_valid);
    if (drive_state_ != DriveState::Active || !valid) {
        if (drive_state_ == DriveState::Active && control_time_ - armed_at_ > startup_timeout_)
            Fault("required motor observations are unavailable");
        torque_command_.fill(0);
        motor_targets_.fill({});
        holding_ = false;
        return;
    }

    if (requested_mode_ != mode_ || request_sequence_ != active_request_sequence_ ||
        (requested_mode_ == 3 && requested_tracking_plane_ != tracking_->plane())) {
        if (mode_ == 6 && zero_set_) {
            zero_set_->Cancel();
            zero_status_ = zero_set_->telemetry();
        }
        if (requested_mode_ == 6) {
            zero_set_ = std::make_unique<ZeroSetController>(config_path_);
            zero_started_ = false;
            zero_applied_.fill(false);
            zero_status_ = {};
            zero_status_.phase = ZeroSetPhase::Waiting;
            zero_clock_ = 0;
            zero_last_control_time_ = input_context_.progress_time;
        } else if (requested_mode_ == 0)
            homing_->Start(control_time_, joint_position_.data());
        else if (requested_mode_ == 1)
            gravity_->Start(joint_position_.data());
        else if (requested_mode_ == 2)
            magnetic_->Start(control_time_, toe_position_);
        else if (requested_mode_ == 3)
            tracking_->Start(
                control_time_, toe_position_, joint_position_, requested_tracking_plane_);
        else if (requested_mode_ == 4) {
            ball_->Start(joint_position_);
            active_ball_target_sequence_ = ball_target_sequence_;
        } else {
            position_reference_ = joint_position_;
            damping_toe_reference_.fill(0);
            damping_reference_valid_ = false;
            std::cout << "[control] DAMPING kp=0 kd=1; awaiting initial model sample" << std::endl;
        }

        mode_ = requested_mode_;
        active_request_sequence_ = request_sequence_;
    }

    holding_ = false;

    if (mode_ == 6) {
        ControlZeroSet();
        return;
    }

    if (mode_ == 0) {
        homing_->Compute(control_time_,
            joint_position_.data(),
            joint_velocity_.data(),
            torque_command_.data(),
            position_reference_.data());
        holding_ = homing_->Reference(control_time_).holding;
    } else if (mode_ == 1) {
        gravity_->Compute(
            model_backend_->state(), torque_command_.data(), position_reference_.data());
    } else if (mode_ == 2) {
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
    } else if (mode_ == 3) {
        tracking_->Compute(control_time_,
            joint_position_,
            joint_velocity_,
            gravity_torque_,
            jacobian_,
            toe_position_,
            torque_command_);

        position_reference_ = tracking_->joint_reference();
    } else if (mode_ == 4) {
        if (ball_target_sequence_ != active_ball_target_sequence_) {
            ball_->SetBallPosition(requested_ball_position_);
            active_ball_target_sequence_ = ball_target_sequence_;
        }

        ball_->Compute(period_ns_ / 1e9,
            joystick_direction_,
            joystick_l2_,
            joystick_r2_,
            joystick_connected_,
            joint_position_,
            joint_velocity_,
            gravity_torque_,
            torque_command_);
        position_reference_ = ball_->joint_reference();
    } else {
        if (!damping_reference_valid_ && model_sequence_) {
            for (double value : toe_position_)
                if (!std::isfinite(value))
                    throw std::runtime_error("non-finite damping entry toe position");

            position_reference_ = joint_position_;
            damping_toe_reference_ = toe_position_;
            damping_reference_valid_ = true;
            std::cout << "[control] DAMPING target captured: [" << damping_toe_reference_[0] << ", "
                      << damping_toe_reference_[1] << ", " << damping_toe_reference_[2]
                      << "] m (no position stiffness)" << std::endl;
        }

        for (int i = 0; i < 3; ++i)
            torque_command_[i] = -joint_velocity_[i];
    }

    for (int i = 0; i < 3; ++i)
        torque_command_[i] = std::clamp(torque_command_[i], -torque_limits_[i], torque_limits_[i]);

    if (mode_ == 5) {
        // Pure MIT damping has no position surrogate and never divides by kp=0.
        for (int i = 0; i < 3; ++i)
            motor_targets_[i] = {position_reference_[i] - encoder_origin_[i],
                joint_velocity_[i] + torque_command_[i],
                0,
                1,
                0};
    } else {
        const auto& kp = mode_ == 0   ? homing_->kp()
                         : mode_ == 1 ? gravity_->kp_motor()
                         : mode_ == 2 ? magnetic_->kp_motor()
                         : mode_ == 3 ? tracking_->kp_motor()
                                      : ball_->kp_motor();
        const auto& kd = mode_ == 0   ? homing_->kd()
                         : mode_ == 1 ? gravity_->kd_motor()
                         : mode_ == 2 ? magnetic_->kd_motor()
                         : mode_ == 3 ? tracking_->kd_motor()
                                      : ball_->kd_motor();
        const auto home = homing_->Reference(control_time_);

        for (int i = 0; i < 3; ++i) {
            // Remove the motor D term already included in torque_command_, then
            // represent the resulting effort as an actual MIT position target.
            const double target_velocity = mode_ == 0 ? home.velocity[i] : 0;
            const double position =
                joint_position_[i] +
                (torque_command_[i] - kd[i] * (target_velocity - joint_velocity_[i])) / kp[i];
            motor_targets_[i] = {position - encoder_origin_[i], target_velocity, kp[i], kd[i], 0};
        }
    }
}

bool TaskController::ZeroComplete() const {
    return std::all_of(
        zero_status_.calibrated.begin(), zero_status_.calibrated.end(), [](bool value) {
            return value;
        });
}

void TaskController::ControlZeroSet() {
    const double progress = input_context_.progress_time;
    zero_clock_ += std::max(0.0, progress - zero_last_control_time_);
    zero_last_control_time_ = progress;
    if (input_context_.calibration_blocked && zero_status_.phase != ZeroSetPhase::Complete &&
        zero_status_.phase != ZeroSetPhase::Fault) {
        zero_set_->Fail(ZeroSetFailure::ExternalContact);
        zero_status_ = zero_set_->telemetry();
    }

    if (!zero_started_ && zero_ready_ && zero_status_.phase != ZeroSetPhase::Fault) {
        zero_set_->Start(zero_clock_, encoder_position_);
        zero_started_ = true;
    }
    if (!zero_started_) {
        if (zero_clock_ > 5.0 && zero_status_.phase != ZeroSetPhase::Fault) {
            zero_set_->Fail(ZeroSetFailure::FeedbackStale);
            zero_status_ = zero_set_->telemetry();
        }
        torque_command_.fill(0);
        motor_targets_.fill({});
        position_reference_ = joint_position_;
        if (zero_status_.phase == ZeroSetPhase::Fault)
            Fault(ZeroSetFailureName(zero_status_.failure));
        return;
    }

    zero_set_->Update(zero_clock_,
        encoder_position_,
        joint_velocity_,
        motor_torque_,
        previous_received_sequence_);
    zero_status_ = zero_set_->telemetry();
    holding_ = zero_status_.phase == ZeroSetPhase::Complete;
    const bool failed = zero_status_.phase == ZeroSetPhase::Fault;
    if (failed)
        Fault(ZeroSetFailureName(zero_status_.failure));

    for (int i = 0; i < 3; ++i) {
        if (zero_status_.calibrated[i] && !zero_applied_[i]) {
            encoder_origin_[i] = zero_status_.offset[i];
            joint_position_[i] = encoder_position_[i] + encoder_origin_[i];
            zero_applied_[i] = true;
        }
        position_reference_[i] = zero_set_->reference()[i] + encoder_origin_[i];
        const double kp = zero_set_->kp()[i], kd = zero_set_->kd()[i];
        const double limit = std::min(torque_limits_[i], zero_set_->effort_limit()[i]);
        torque_command_[i] =
            failed ? 0
                   : std::clamp(kp * (zero_set_->reference()[i] - encoder_position_[i]) -
                                    kd * joint_velocity_[i],
                         -limit,
                         limit);
        // The encoder frame is unchanged: calibration only changes the runtime
        // encoder->joint mapping. No EEPROM/ROM write or C3 rebase is sent.
        motor_targets_[i] = {
            encoder_position_[i] + (torque_command_[i] + kd * joint_velocity_[i]) / kp,
            0,
            kp,
            kd,
            0};
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
        motors.command_enabled[i] = motor_commands_enabled_ && enabled_axes_[i];
        motors.debug_torque[i] = torque_command_[i];
        motors.reference[i] = position_reference_[i];
        if (zero_applied_[i]) {
            motors.position_offset[i] = encoder_origin_[i];
            motors.coordinate_valid[i] = true;
        }
    }

    motors.clear_fault_sequence = clear_fault_sequence_;
    auto& control = shm_utils::Next(shm.control, shm.control_sequence);
    control.command_sequence = motors.command_sequence;
    control.published_ns = motors.command_published_ns;
    control.input_model_sequence = model_sequence_;
    control.input_model_published_ns = model_published_ns_;
    control.active_motion_sequence = active_request_sequence_;
    control.control_time = control_time_;
    control.mode = mode_ < 0 ? requested_mode_ : mode_;
    control.drive_state = drive_state_;
    control.active_drive_request_sequence = drive_request_sequence_;
    control.drive_fault_reason.fill(0);
    std::copy_n(fault_reason_.begin(),
        std::min(fault_reason_.size(), control.drive_fault_reason.size() - 1),
        control.drive_fault_reason.begin());
    control.holding = holding_;
    control.zero_set = zero_status_;
    control.zero_set.required = zero_required_;
    control.damping_toe_reference = mode_ == 5 ? damping_toe_reference_ : std::array<double, 3>{};
    control.damping_reference_valid = mode_ == 5 && damping_reference_valid_;
    control.magnetic = mode_ == 2 ? magnetic_->telemetry() : MagneticTelemetry{};
    control.tracking_position_reference =
        mode_ == 3 ? tracking_->toe_reference() : std::array<double, 3>{};
    control.tracking_velocity_reference =
        mode_ == 3 ? tracking_->toe_velocity_reference() : std::array<double, 3>{};
    control.tracking_torque_saturated = mode_ == 3 && tracking_->torque_saturated();
    control.tracking_plane = tracking_->plane();
    control.tracking_method = tracking_->method();
    control.tracking_acceleration_reference =
        mode_ == 3 ? tracking_->toe_acceleration_reference() : std::array<double, 3>{};
    control.tracking_joint_velocity_reference =
        mode_ == 3 ? tracking_->joint_velocity_reference() : std::array<double, 3>{};
    control.tracking_joint_acceleration_reference =
        mode_ == 3 ? tracking_->joint_acceleration_reference() : std::array<double, 3>{};
    control.tracking_inverse_dynamics_torque =
        mode_ == 3 ? tracking_->inverse_dynamics_torque() : std::array<double, 3>{};
    control.tracking_ctc_torque = mode_ == 3 ? tracking_->ctc_torque() : std::array<double, 3>{};
    control.tracking_ilc_state = mode_ == 3 ? tracking_->ilc().state() : TrackingILCState::Off;
    control.tracking_ilc_torque = mode_ == 3 ? tracking_->ilc().torque() : std::array<double, 3>{};
    control.tracking_ilc_updates = mode_ == 3 ? tracking_->ilc().updates() : 0;
    control.tracking_ilc_skipped_cycles = mode_ == 3 ? tracking_->ilc().skipped_cycles() : 0;
    control.tracking_ilc_limited = mode_ == 3 && tracking_->ilc().limited();
    control.tracking_ilc_last_rmse_m = mode_ == 3 ? tracking_->ilc().last_rmse_m() : 0;
    control.tracking_idff_enabled = mode_ == 3 && tracking_->inverse_dynamics_enabled() &&
                                    !tracking_->ik_failed() && !tracking_->checking();
    control.tracking_ik_failed = mode_ == 3 && tracking_->ik_failed();
    control.tracking_checking = mode_ == 3 && tracking_->checking();
    control.tracking_check_progress = mode_ == 3 ? tracking_->check_progress() : 0;
    control.tracking_ik_error_m = mode_ == 3 ? tracking_->ik_error() : 0;
    control.ball_position = mode_ == 4 ? ball_->ball_position() : std::array<double, 3>{};
    control.ball_toe_reference = mode_ == 4 ? ball_->toe_reference() : std::array<double, 3>{};
    control.ball_nearest_position =
        mode_ == 4 ? ball_->nearest_position() : std::array<double, 3>{};
    control.ball_workspace_limited = mode_ == 4 && ball_->workspace_limited();
    control.ball_entering_workspace = mode_ == 4 && ball_->entering_workspace();
    control.ball_torque_saturated = mode_ == 4 && ball_->torque_saturated();
    control.ball_speed_m_s = mode_ == 4 ? ball_->ball_speed() : 0;

    ++shm.motor_sequence;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
}

void TaskController::Release() {
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
