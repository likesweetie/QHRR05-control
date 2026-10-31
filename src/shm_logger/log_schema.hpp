#pragma once

#include "shm_logger/shm_snapshot_reader.hpp"
#include "data_tamer_v2/channel.hpp"
#include <optional>

// Logging-only reflection. SHM and control headers have no DataTamer dependency.

template <> struct DataTamer2::TypeDefinitionTrait<MitCommand> {
    template <typename Add> static std::string_view define(MitCommand& value, Add& add) {
        add("position", &value.position);
        add("velocity", &value.velocity);
        add("kp", &value.kp);
        add("kd", &value.kd);
        add("tau_ff", &value.tau_ff);
        return "MitCommand";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<MotorFeedback> {
    template <typename Add> static std::string_view define(MotorFeedback& value, Add& add) {
        add("position", &value.position);
        add("velocity", &value.velocity);
        add("current", &value.current);
        add("temperature", &value.temperature);
        return "MotorFeedback";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<MagneticTelemetry> {
    template <typename Add> static std::string_view define(MagneticTelemetry& value, Add& add) {
        add("phase", &value.phase);
        add("reason", &value.reason);
        add("observer_valid", &value.observer_valid);
        add("torque_saturated", &value.torque_saturated);
        add("event_sequence", &value.event_sequence);
        add("estimated_normal_force", &value.estimated_normal_force);
        add("target_force", &value.target_force);
        add("force_command", &value.force_command);
        add("displacement", &value.displacement);
        add("minimum_singular_value", &value.minimum_singular_value);
        add("event_time", &value.event_time);
        add("estimated_force", &value.estimated_force);
        add("external_torque", &value.external_torque);
        add("normal", &value.normal);
        add("toe_reference", &value.toe_reference);
        return "MagneticTelemetry";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<MotorData> {
    template <typename Add> static std::string_view define(MotorData& value, Add& add) {
        add("command", &value.command);
        add("command_enabled", &value.command_enabled);
        add("reference", &value.reference);
        add("debug_torque", &value.debug_torque);
        add("feedback", &value.feedback);
        add("motor_torque", &value.motor_torque);
        add("received_ns", &value.received_ns);
        add("received_sequence", &value.received_sequence);
        add("tx_frames", &value.tx_frames);
        add("rx_frames", &value.rx_frames);
        add("invalid_frames", &value.invalid_frames);
        add("profile_verified", &value.profile_verified);
        add("enable_requested", &value.enable_requested);
        add("enabled", &value.enabled);
        add("position_offset", &value.position_offset);
        add("command_sequence", &value.command_sequence);
        add("command_published_ns", &value.command_published_ns);
        add("feedback_sequence", &value.feedback_sequence);
        add("feedback_published_ns", &value.feedback_published_ns);
        add("consumed_command_sequence", &value.consumed_command_sequence);
        add("last_tx_ns", &value.last_tx_ns);
        add("cycles", &value.cycles);
        add("socket_errors", &value.socket_errors);
        add("sample_dt", &value.sample_dt);
        add("running", &value.running);
        return "MotorData";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<ModelData> {
    template <typename Add> static std::string_view define(ModelData& value, Add& add) {
        add("published_ns", &value.published_ns);
        add("gravity_torque", &value.gravity_torque);
        add("bias_torque", &value.bias_torque);
        add("passive_torque", &value.passive_torque);
        add("friction_limit", &value.friction_limit);
        add("mass", &value.mass);
        add("jacobian", &value.jacobian);
        add("toe_position", &value.toe_position);
        add("joint_min", &value.joint_min);
        add("joint_max", &value.joint_max);
        return "ModelData";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<ZeroSetTelemetry> {
    template <typename Add> static std::string_view define(ZeroSetTelemetry& value, Add& add) {
        add("phase", &value.phase);
        add("failure", &value.failure);
        add("axis", &value.axis);
        add("required", &value.required);
        add("calibrated", &value.calibrated);
        add("offset", &value.offset);
        add("travel_rad", &value.travel_rad);
        add("elapsed_sec", &value.elapsed_sec);
        return "ZeroSetTelemetry";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<ControlData> {
    template <typename Add> static std::string_view define(ControlData& value, Add& add) {
        add("requested_mode", &value.requested_mode);
        add("requested_tracking_plane", &value.requested_tracking_plane);
        add("target_force", &value.target_force);
        add("motion_sequence", &value.motion_sequence);
        add("mode", &value.mode);
        add("holding", &value.holding);
        add("zero_set", &value.zero_set);
        add("damping_toe_reference", &value.damping_toe_reference);
        add("damping_reference_valid", &value.damping_reference_valid);
        add("control_time", &value.control_time);
        add("command_sequence", &value.command_sequence);
        add("published_ns", &value.published_ns);
        add("active_motion_sequence", &value.active_motion_sequence);
        add("input_model_sequence", &value.input_model_sequence);
        add("input_model_published_ns", &value.input_model_published_ns);
        add("magnetic", &value.magnetic);
        add("tracking_position_reference", &value.tracking_position_reference);
        add("tracking_velocity_reference", &value.tracking_velocity_reference);
        add("tracking_torque_saturated", &value.tracking_torque_saturated);
        add("ball_target_sequence", &value.ball_target_sequence);
        add("requested_ball_position", &value.requested_ball_position);
        add("ball_position", &value.ball_position);
        add("ball_toe_reference", &value.ball_toe_reference);
        add("ball_nearest_position", &value.ball_nearest_position);
        add("ball_workspace_limited", &value.ball_workspace_limited);
        add("ball_entering_workspace", &value.ball_entering_workspace);
        add("ball_torque_saturated", &value.ball_torque_saturated);
        add("ball_speed_m_s", &value.ball_speed_m_s);
        add("joystick_l2", &value.joystick_l2);
        add("joystick_r2", &value.joystick_r2);
        add("joystick_connected", &value.joystick_connected);
        add("tracking_plane", &value.tracking_plane);
        add("tracking_method", &value.tracking_method);
        add("tracking_ctc_torque", &value.tracking_ctc_torque);
        add("tracking_ilc_state", &value.tracking_ilc_state);
        add("tracking_ilc_torque", &value.tracking_ilc_torque);
        add("tracking_ilc_updates", &value.tracking_ilc_updates);
        add("tracking_ilc_skipped_cycles", &value.tracking_ilc_skipped_cycles);
        add("tracking_ilc_limited", &value.tracking_ilc_limited);
        add("tracking_ilc_last_rmse_m", &value.tracking_ilc_last_rmse_m);
        add("tracking_acceleration_reference", &value.tracking_acceleration_reference);
        add("tracking_joint_velocity_reference", &value.tracking_joint_velocity_reference);
        add("tracking_joint_acceleration_reference", &value.tracking_joint_acceleration_reference);
        add("tracking_inverse_dynamics_torque", &value.tracking_inverse_dynamics_torque);
        add("tracking_idff_enabled", &value.tracking_idff_enabled);
        add("tracking_ik_failed", &value.tracking_ik_failed);
        add("tracking_checking", &value.tracking_checking);
        add("tracking_check_progress", &value.tracking_check_progress);
        add("tracking_ik_error_m", &value.tracking_ik_error_m);
        add("joystick_axes", &value.joystick_axes);
        add("joystick_buttons", &value.joystick_buttons);
        add("joystick_sequence", &value.joystick_sequence);
        add("joystick_published_ns", &value.joystick_published_ns);
        return "ControlData";
    }
};

template <> struct DataTamer2::TypeDefinitionTrait<SimulationData> {
    template <typename Add> static std::string_view define(SimulationData& value, Add& add) {
        add("published_ns", &value.published_ns);
        add("run_id", &value.run_id);
        add("sequence", &value.sequence);
        add("transport_drops", &value.transport_drops);
        add("finished", &value.finished);
        add("time", &value.time);
        add("position_gt", &value.position_gt);
        add("torque_gt", &value.torque_gt);
        add("toe_position", &value.toe_position);
        add("contact_force", &value.contact_force);
        add("contact_torque", &value.contact_torque);
        add("object_position", &value.object_position);
        add("object_quaternion", &value.object_quaternion);
        add("velocity_gt", &value.velocity_gt);
        add("paused", &value.paused);
        add("applied_input_sequence", &value.applied_input_sequence);
        add("applied_pause_sequence", &value.applied_pause_sequence);
        add("applied_step_sequence", &value.applied_step_sequence);
        add("perturb_owner", &value.perturb_owner);
        add("perturb_active", &value.perturb_active);
        add("perturb_wrench", &value.perturb_wrench);
        add("contact", &value.contact);
        add("contact_event_sequence", &value.contact_event_sequence);
        add("contact_event_time", &value.contact_event_time);
        add("require_zero_position_setup", &value.require_zero_position_setup);
        return "SimulationData";
    }
};

namespace qhrr05::logging {
std::optional<DataTamer2::RegistrationID> RegisterState(
    DataTamer2::LogChannel& channel, LogFrame& frame, bool simulation);
}  // namespace qhrr05::logging
