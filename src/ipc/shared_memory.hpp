#pragma once

#include "ipc/control_types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <pthread.h>
#include <time.h>

// Shared-memory protocol. Increment the ABI version when layout or field semantics change.
inline constexpr std::uint64_t kShmMagic = 0x5148525253484D31ULL;  // QHRRSHM1
inline constexpr std::uint32_t kShmAbiVersion = 16;
inline constexpr std::size_t kShmBufferCount = 2;
inline constexpr unsigned kShmPermissions = 0600;
inline constexpr clockid_t kShmClock = CLOCK_MONOTONIC;

// Canonical motor data for HR / HP / KP; every array follows that axis order.
struct MotorData {
    // Controller-owned outputs.
    std::array<MitCommand, 3> command{};
    std::array<bool, 3> command_enabled{};
    std::array<double, 3> reference{}, debug_torque{};

    // HAL-owned feedback and per-axis CAN status.
    std::array<MotorFeedback, 3> feedback{};
    std::array<double, 3> motor_torque{};  // Nm, converted by HAL using verified motor Kt.
    std::array<std::uint64_t, 3> received_ns{}, received_sequence{};
    std::array<std::uint64_t, 3> tx_frames{}, rx_frames{}, invalid_frames{};
    std::array<bool, 3> profile_verified{}, enable_requested{}, enabled{};
    std::array<std::uint32_t, 3> supported{}, valid{};
    std::array<std::uint64_t, 3> measurement_ns{};  // Producer sample clock, for derivatives.
    std::array<bool, 3> coordinate_valid{};  // Controller-owned encoder -> joint calibration.
    TransportFault transport_fault = TransportFault::None;
    std::uint64_t clear_fault_sequence = 0;  // Explicit controller request, never automatic.

    // Joint angle = raw CAN encoder + controller-owned runtime calibration.
    // Simulation bootstrap initializes this once; reset/enable never clear it.
    std::array<double, 3> position_offset{};

    std::uint64_t command_sequence = 0, command_published_ns = 0;  // Controller
    std::uint64_t feedback_sequence = 0, feedback_published_ns = 0;  // HAL (sim in debug)
    std::uint64_t consumed_command_sequence = 0, last_tx_ns = 0, cycles = 0, socket_errors = 0;
    double sample_dt = .001;
    bool running = false;
};

// Controller-owned model information, evaluated by its backend at decoded joint positions.
struct ModelData {
    std::uint64_t published_ns = 0;
    std::uint64_t input_measurement_sequence = 0;
    bool valid = false;
    bool acceleration_valid = false;
    std::array<double, 3> joint_acceleration{};
    std::array<double, 3> gravity_torque{}, bias_torque{}, passive_torque{}, friction_limit{};
    std::array<double, 9> mass{}, jacobian{};
    std::array<double, 3> toe_position{}, joint_min{}, joint_max{};
};

struct ControlData {
    int requested_mode =
        5;  // 0: homing, 1: gravity, 2: magnetic, 3: tracking, 4: ball, 5: damping, 6: zero-set.
    // GUI owns requests. Slider edits preserve motion_sequence.
    double target_force = 3;
    TrackingPlane requested_tracking_plane = TrackingPlane::XY;
    std::uint64_t motion_sequence = 0;
    DriveAction requested_drive_action = DriveAction::None;
    int requested_motor_axis = -1;  // Only EnableMotor/DisableMotor use an axis.
    std::uint64_t drive_request_sequence = 0;
    std::array<double, 3> requested_ball_position{};  // Optional explicit target jump.
    std::uint64_t ball_target_sequence = 0;

    int mode = 5;  // Controller owns status and observer output. Startup is damping only.
    bool holding = false;
    DriveState drive_state = DriveState::Disarmed;
    std::uint64_t active_drive_request_sequence = 0;
    std::array<char, 160> drive_fault_reason{};
    ZeroSetTelemetry zero_set{};
    std::array<double, 3>
        damping_toe_reference{};  // Captured on damping entry, no position stiffness.
    bool damping_reference_valid = false;  // False until the first model sample is available.
    double control_time = 0;
    std::uint64_t command_sequence = 0, published_ns = 0, active_motion_sequence = 0;
    std::uint64_t input_model_sequence = 0, input_model_published_ns = 0;  // Logger trace
    MagneticTelemetry magnetic{};
    std::array<double, 3> tracking_position_reference{}, tracking_velocity_reference{};  // m, m/s
    bool tracking_torque_saturated = false;
    TrackingPlane tracking_plane = TrackingPlane::XY;
    TrackingMethod tracking_method = TrackingMethod::InverseDynamics;
    std::array<double, 3> tracking_acceleration_reference{};  // m/s^2
    std::array<double, 3> tracking_joint_velocity_reference{};  // rad/s; q_d is motors.reference
    std::array<double, 3> tracking_joint_acceleration_reference{};  // rad/s^2
    std::array<double, 3> tracking_inverse_dynamics_torque{};  // Nm, before alpha blending/clipping
    std::array<double, 3>
        tracking_ctc_torque{};  // Nm, full CTC output before clipping; zero outside CTC.
    TrackingILCState tracking_ilc_state = TrackingILCState::Off;
    std::array<double, 3>
        tracking_ilc_torque{};  // Nm, bounded learned correction added before final clipping.
    std::uint64_t tracking_ilc_updates = 0, tracking_ilc_skipped_cycles = 0;
    bool tracking_ilc_limited = false;  // Last accepted update reached an ILC step/torque limit.
    double tracking_ilc_last_rmse_m =
        0;  // Last fully sampled lap; NaN until available when enabled.
    bool tracking_idff_enabled = false;
    bool tracking_ik_failed =
        false;  // IK/condition rejection: latched hold until explicit restart.
    bool tracking_checking = false;  // Entry pose is held until the full path is approved.
    double tracking_check_progress = 0;  // Completed fraction of the sampled path, 0..1.
    double tracking_ik_error_m = 0;
    std::array<double, 3> ball_position{}, ball_toe_reference{}, ball_nearest_position{};
    bool ball_workspace_limited = false, ball_entering_workspace = false;
    bool ball_torque_saturated = false;
    double ball_speed_m_s = 0;

    std::array<double, 3> joystick_axes{};  // Optional input owns these fields only.
    std::array<bool, 11> joystick_buttons{};
    std::uint64_t joystick_sequence = 0, joystick_published_ns = 0;
    double joystick_l2 = 0, joystick_r2 = 0;  // Normalized trigger pressure, 0..1.
    bool joystick_connected = false;
};

struct ShmData {
    // The owner sets magic last, after initializing the process-shared synchronization.
    std::uint64_t magic = 0;
    std::uint32_t abi_version = kShmAbiVersion;
    std::uint32_t buffer_count = kShmBufferCount;
    std::uint64_t byte_size = 0;

    pthread_mutex_t mutex;
    pthread_cond_t changed;
    bool stopping = false, failed = false, can_enabled = false;
    bool simulation_available = false;  // Observer/command capability, not a motion input.
    ObservationOrigin observation_origin = ObservationOrigin::Device;

    // All copies/publications hold mutex. Multiwriter channels copy the CURRENT
    // published slot before updating their own fields; never publish a stale bank.
    std::uint64_t motor_sequence = 0, model_sequence = 0, control_sequence = 0;
    std::array<MotorData, kShmBufferCount> motors{};
    std::array<ModelData, kShmBufferCount> model{};
    std::array<ControlData, kShmBufferCount> control{};
};

inline constexpr std::size_t kShmByteSize = sizeof(ShmData);
