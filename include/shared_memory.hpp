#pragma once

#include <array>
#include <cstdint>
#include <pthread.h>
#include "motor_can/openrobot_protocol.hpp"

enum class MagneticPhase : int { Free, Candidate, Following, Releasing };
enum class MagneticReason : int {
    None,
    NoMeasurement,
    ObserverInvalid,
    Workspace,
    JointLimit,
    StrokeLimit,
    ContactLost
};

inline const char* MagneticPhaseName(MagneticPhase phase) {
    switch (phase) {
    case MagneticPhase::Free:
        return "FREE";
    case MagneticPhase::Candidate:
        return "CONTACT CANDIDATE";
    case MagneticPhase::Following:
        return "FORCE FOLLOWING";
    case MagneticPhase::Releasing:
        return "RELEASE PENDING";
    }

    return "INVALID PHASE";
}

inline const char* MagneticReasonName(MagneticReason reason) {
    switch (reason) {
    case MagneticReason::None:
        return "none";
    case MagneticReason::NoMeasurement:
        return "no measurement";
    case MagneticReason::ObserverInvalid:
        return "observer invalid/singular";
    case MagneticReason::Workspace:
        return "workspace limit";
    case MagneticReason::JointLimit:
        return "joint limit";
    case MagneticReason::StrokeLimit:
        return "following distance limit";
    case MagneticReason::ContactLost:
        return "contact lost";
    }

    return "invalid reason";
}

struct MagneticTelemetry {
    MagneticPhase phase = MagneticPhase::Free;
    MagneticReason reason = MagneticReason::None;
    bool observer_valid = false, torque_saturated = false;
    std::uint64_t event_sequence = 0;
    double estimated_normal_force = 0, target_force = 0, force_command = 0;
    double displacement = 0, minimum_singular_value = 0, event_time = 0;
    std::array<double, 3> estimated_force{}, external_torque{}, normal{}, toe_reference{};
};

// Canonical motor data for HR / HP / KP; every array follows that axis order.
struct MotorData {
    // Controller-owned outputs.
    std::array<openrobot::PositionTarget, 3> command{};
    std::array<double, 3> reference{}, debug_torque{};

    // HAL-owned feedback and per-axis CAN status.
    std::array<openrobot::Feedback, 3> feedback{};
    std::array<std::uint64_t, 3> received_ns{}, received_sequence{};
    std::array<std::uint64_t, 3> tx_frames{}, rx_frames{}, invalid_frames{};
    std::array<bool, 3> profile_verified{}, enable_requested{}, enabled{};

    // Simulation bootstrap, then runtime calibration.
    std::array<double, 3> position_offset{};

    openrobot::MitProfile profile{};
    std::uint64_t command_sequence = 0, command_published_ns = 0;  // Controller
    std::uint64_t feedback_sequence = 0, feedback_published_ns = 0;  // HAL (sim in debug)
    std::uint64_t consumed_command_sequence = 0, last_tx_ns = 0, cycles = 0, socket_errors = 0;
    double sample_dt = .001;
    bool running = false;
};

// Simulator-owned model information, evaluated at decoded joint positions.
struct ModelData {
    std::uint64_t published_ns = 0;
    std::array<double, 3> gravity_torque{}, bias_torque{}, passive_torque{}, friction_limit{};
    std::array<double, 9> mass{}, jacobian{};
    std::array<double, 3> toe_position{}, joint_min{}, joint_max{};
};

struct ControlData {
    int requested_mode = 0;  // GUI owns requests. Slider edits preserve motion_sequence.
    double target_force = 3;
    std::uint64_t motion_sequence = 0;

    int mode = 0;  // Controller owns status and observer output.
    bool holding = false;
    double control_time = 0;
    std::uint64_t command_sequence = 0, published_ns = 0, active_motion_sequence = 0;
    std::uint64_t input_model_sequence = 0, input_model_published_ns = 0;  // Logger trace
    MagneticTelemetry magnetic{};

    std::array<double, 3> joystick_axes{};  // Optional aux_reader owns these fields only.
    std::array<bool, 11> joystick_buttons{};
    std::uint64_t joystick_sequence = 0, joystick_published_ns = 0;
};

// Physical truth and object/contact events stay on the simulation SHM path.
struct SimulationData {
    std::uint64_t published_ns = 0, run_id = 0;
    double time = 0;
    std::array<double, 3> position_gt{}, torque_gt{}, toe_position{};
    std::array<double, 3> contact_force{}, contact_torque{}, object_position{};
    bool contact = false;
    std::uint64_t contact_event_sequence = 0;
    double contact_event_time = 0;
};

struct ShmData {
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    bool stopping = false, failed = false, simulation_finished = false, can_enabled = false;

    // All copies/publications hold mutex. Multiwriter channels copy the CURRENT
    // published slot before updating their own fields; never publish a stale bank.
    std::uint64_t motor_sequence = 0, model_sequence = 0, control_sequence = 0,
                  simulation_sequence = 0;
    std::array<MotorData, 2> motors{};
    std::array<ModelData, 2> model{};
    std::array<ControlData, 2> control{};
    std::array<SimulationData, 2> simulation{};
};
