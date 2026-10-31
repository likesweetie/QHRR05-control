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
    : shm_(shm_name), input_(shm_name), safety_(config_path), simulation_channel_(shm_name, true),
      without_can_(without_can), config_path_(config_path),
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

        if (shm.can_enabled == without_can_ ||
            (!without_can_ && (shm.failed || shm.stopping || !motors.running)))
            throw std::runtime_error("SHM transport does not match controller mode");
        zero_status_ = shm.control[shm.control_sequence % kShmBufferCount].zero_set;
    }

    const auto config = YAML::LoadFile(config_path);
    torque_limits_ = ReadControlTorqueLimits(config);
    const double frequency = config["control"]["frequency_hz"].as<double>();

    if (!std::isfinite(frequency) || frequency <= 0)
        throw std::runtime_error("control.frequency_hz must be positive and finite");

    const double period = 1e9 / frequency;

    if (period < 1 || period >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("control.frequency_hz has an unrepresentable period");

    period_ns_ = static_cast<std::uint64_t>(period);
    started_ns_ = next_cycle_ns_ = shm_utils::MonotonicNs();
}

bool TaskController::ReadInput() {
    try {
        simulation_channel_.Poll();
        last_simulation_error_.clear();
    } catch (const std::exception& error) {
        if (last_simulation_error_ != error.what())
            std::cerr << "[simulation-ipc] ERROR: " << error.what()
                      << "; simulation commands unavailable; hardware control remains active\n";
        last_simulation_error_ = error.what();
    }
    const auto& simulation = simulation_channel_.Latest();
    const bool auto_zero = !zero_policy_seen_ && simulation.published_ns &&
                           simulation.require_zero_position_setup && !ZeroComplete();
    const bool resume_calibrated = !zero_policy_seen_ && simulation.published_ns &&
                                   simulation.require_zero_position_setup && ZeroComplete();
    if (simulation.published_ns) {
        zero_policy_seen_ = true;
        zero_required_ = simulation.require_zero_position_setup;
    }
    const auto command = input_.Receive();
    std::string rejection;
    std::uint64_t approved_sequence = 0;
    std::optional<SimulationCommand> simulation_pending;
    bool calibration_rejected = false;
    std::array<std::uint64_t, 3> received_sequence, received_ns;
    bool measured = false;
    {
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);

        if (shm.failed)
            throw std::runtime_error("SHM stopped after a process failure");

        if (shm.stopping)
            return false;

        const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        measured = without_can_ ? motors.feedback_sequence != 0
                                : std::all_of(motors.received_sequence.begin(),
                                      motors.received_sequence.end(),
                                      [](auto sequence) {
                                          return sequence != 0;
                                      });
        motor_commands_enabled_ = !without_can_;

        for (int i = 0; i < 3; ++i) {
            encoder_origin_[i] = motors.position_offset[i];
            encoder_position_[i] = motors.feedback[i].position;
            joint_position_[i] = motors.position_offset[i] + motors.feedback[i].position;
            joint_velocity_[i] = motors.feedback[i].velocity;
            motor_torque_[i] = motors.motor_torque[i];
            motor_commands_enabled_ &= motors.profile_verified[i];
        }

        received_sequence = motors.received_sequence;
        received_ns = motors.received_ns;
        measurement_dt_ = motors.sample_dt;
        zero_ready_ = !without_can_ && measured && motor_commands_enabled_ &&
                      std::all_of(motors.enabled.begin(), motors.enabled.end(), [](bool v) {
                          return v;
                      });

        auto candidate = shm.control[shm.control_sequence % kShmBufferCount];
        if (resume_calibrated && candidate.requested_mode == 6) {
            // A control process restart must not erase a completed session calibration.
            candidate.requested_mode = 5;
            ++candidate.motion_sequence;
            auto& restored = shm_utils::Next(shm.control, shm.control_sequence);
            restored.requested_mode = 5;
            restored.motion_sequence = candidate.motion_sequence;
            ++shm.control_sequence;
            std::cout << "[zero-set] using completed runtime calibration; restarting in DAMPING"
                      << std::endl;
        }
        if (auto_zero) {
            candidate.requested_mode = 6;
            ++candidate.motion_sequence;
            auto& approved = shm_utils::Next(shm.control, shm.control_sequence);
            approved.requested_mode = candidate.requested_mode;
            approved.motion_sequence = candidate.motion_sequence;
            ++shm.control_sequence;
            requested_mode_ = 6;
            request_sequence_ = candidate.motion_sequence;
            model_published_ns_ = 0;
            auto& invalid_model = shm_utils::Next(shm.model, shm.model_sequence);
            invalid_model = {};
            ++shm.model_sequence;
            std::cout << "[zero-set] startup calibration required; normal motion locked"
                      << std::endl;
        }

        if (command) {
            if (command->set_mode) {
                candidate.requested_mode = command->mode;
                candidate.requested_tracking_plane = command->plane;
                ++candidate.motion_sequence;
            } else if (command->simulation.action == SimulationAction::None)
                candidate.target_force = command->force;
        }

        try {
            safety_.Validate(candidate, shm.stopping, shm.failed);

            if (candidate.requested_mode == 6 && without_can_) {
                calibration_rejected = true;
                throw std::runtime_error(
                    "zero-set requires motor encoder/torque feedback; --without-can is unsupported");
            }
            if (zero_required_ && !ZeroComplete() && candidate.requested_mode != 5 &&
                candidate.requested_mode != 6) {
                calibration_rejected = true;
                throw std::runtime_error(
                    "zero-set is incomplete: use Z to calibrate or D to cancel");
            }

            if (command && !model_published_ns_ &&
                !(command->set_mode && (command->mode == 5 || command->mode == 6)) &&
                command->simulation.action == SimulationAction::None)
                throw std::runtime_error("safety rejected request: model is not ready");
            if (command && command->simulation.action != SimulationAction::None)
                safety_.ValidateSimulation(command->simulation,
                    simulation_channel_.Latest(),
                    simulation_approval_,
                    shm_utils::MonotonicNs());
        } catch (const std::runtime_error& error) {
            rejection = error.what();
        }

        // Native/joystick inputs already occupy the request bank. Reject them
        // conclusively; never execute a rejected motion later when calibration ends.
        if (calibration_rejected && !command) {
            auto& restored = shm_utils::Next(shm.control, shm.control_sequence);
            restored.requested_mode = requested_mode_;
            restored.requested_tracking_plane = requested_tracking_plane_;
            restored.motion_sequence = request_sequence_;
            ++shm.control_sequence;
        }

        if (rejection.empty()) {
            if (command && command->simulation.action != SimulationAction::None) {
                auto request = command->simulation;
                const auto& state = simulation_channel_.Latest();
                if (request.action == SimulationAction::Pause) {
                    const bool paused =
                        simulation_approval_.pause_sequence > state.applied_pause_sequence
                            ? simulation_approval_.paused
                            : state.paused;
                    request.pause = request.pause == 2 ? !paused : request.pause;
                }
                simulation_pending = request;
            } else if (command) {
                auto& approved = shm_utils::Next(shm.control, shm.control_sequence);
                approved.requested_mode = candidate.requested_mode;
                approved.requested_tracking_plane = candidate.requested_tracking_plane;
                approved.motion_sequence = candidate.motion_sequence;
                approved.target_force = candidate.target_force;
                ++shm.control_sequence;
                shm_utils::Notify(shm);
            }

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
            if (!command || command->simulation.action == SimulationAction::None)
                approved_sequence = request_sequence_;
        }
    }

    if (simulation_pending) {
        const auto& request = *simulation_pending;
        try {
            approved_sequence = simulation_channel_.SendApproved(request);
            if (request.action == SimulationAction::Pause) {
                simulation_approval_.paused = request.pause;
                simulation_approval_.pause_sequence = approved_sequence;
                simulation_approval_.perturb.active = 0;
            } else if (request.action == SimulationAction::Reset) {
                simulation_approval_.perturb.active = 0;
            } else if (request.action == SimulationAction::Perturb) {
                simulation_approval_.perturb = request.perturb;
                simulation_approval_.perturb_expires_ns =
                    request.perturb.active ? shm_utils::MonotonicNs() + kPerturbLeaseNs : 0;
            }
        } catch (const std::runtime_error& error) {
            rejection = error.what();
        }
    }

    if (command)
        input_.Reply(*command, rejection, approved_sequence);

    if (!rejection.empty() && rejection != last_input_rejection_)
        std::cerr << "[safety] " << rejection << "; retaining last approved input\n";

    last_input_rejection_ = rejection;

    if (requested_mode_ < 0 || requested_mode_ > 6)
        throw std::runtime_error("invalid controller mode");

    TrackingPlaneAxes(requested_tracking_plane_);

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

        if (without_can_) {
            measurement_sequence_ = *std::max_element(
                previous_received_sequence_.begin(), previous_received_sequence_.end());
        } else {
            ++measurement_sequence_;
        }
    }

    if (measured && (!zero_required_ || ZeroComplete()) && (fresh || !model_published_ns_)) {
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
        // Publish the same measured-state model for hardware and simulation observers.
        // No solver, model update or allocation occurs while holding this mutex.
        auto& shm = *shm_.get();
        shm_utils::Lock lock(shm);
        auto& model = shm_utils::Next(shm.model, shm.model_sequence);
        model.published_ns = shm_utils::MonotonicNs();
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
        shm_utils::Notify(shm);
    }
    return true;
}

void TaskController::Control() {
    control_time_ = (shm_utils::MonotonicNs() - started_ns_) / 1e9;

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
            zero_last_control_time_ = control_time_;
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

    if (!without_can_ && mode_ == 5) {
        // Pure MIT damping has no position surrogate and never divides by kp=0.
        for (int i = 0; i < 3; ++i)
            motor_targets_[i] = {position_reference_[i] - encoder_origin_[i],
                joint_velocity_[i] + torque_command_[i],
                0,
                1,
                0};
    } else if (!without_can_) {
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
    const auto& simulation = simulation_channel_.Latest();
    const auto now = shm_utils::MonotonicNs();
    const bool paused = simulation_channel_.Connected() && !simulation.finished &&
                        simulation.published_ns && now >= simulation.published_ns &&
                        now - simulation.published_ns < 500000000ULL && simulation.paused;
    if (!paused)
        zero_clock_ += std::max(0.0, control_time_ - zero_last_control_time_);
    zero_last_control_time_ = control_time_;

    // Simulation truth can veto an unsafe external contact, never establish the
    // encoder zero. Hardware detection still uses only measured torque/velocity.
    if (simulation.published_ns && simulation.contact &&
        zero_status_.phase != ZeroSetPhase::Complete && zero_status_.phase != ZeroSetPhase::Fault) {
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
            motor_commands_enabled_ = false;
        return;
    }

    if (!paused)
        zero_set_->Update(zero_clock_,
            encoder_position_,
            joint_velocity_,
            motor_torque_,
            previous_received_sequence_);
    zero_status_ = zero_set_->telemetry();
    holding_ = zero_status_.phase == ZeroSetPhase::Complete;
    const bool failed = zero_status_.phase == ZeroSetPhase::Fault;
    if (failed)
        motor_commands_enabled_ = false;

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
        motors.command_enabled[i] = motor_commands_enabled_;
        motors.debug_torque[i] = torque_command_[i];
        motors.reference[i] = position_reference_[i];
        if (zero_applied_[i])
            motors.position_offset[i] = encoder_origin_[i];
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
