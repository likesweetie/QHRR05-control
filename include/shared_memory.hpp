#pragma once

#include <array>
#include <cstdint>
#include <pthread.h>

// Fixed qhrr-05 joint order: FL hip, thigh, calf. All accesses use ShmData::mutex.
struct ShmState {
    std::uint64_t sequence = 0;
    std::uint64_t published_ns = 0;

    std::array<double, 3> position{}, velocity{}, gravity_torque{};

    // Generic, time-aligned model/encoder feedback. No contact/constraint truth.
    std::uint64_t measurement_sequence = 0;
    double measurement_dt = 0;

    std::array<double, 3> acceleration{}, motor_torque{}, bias_torque{}, passive_torque{},
        friction_limit{};
    std::array<double, 9> mass{}, jacobian{};  // Row-major, J maps dq to world toe velocity.
    std::array<double, 3> toe_position{}, joint_min{}, joint_max{};
};

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

struct ShmControlRequest {
    std::uint64_t sequence = 0;
    int mode = 0;  // 0=homing, 1=gravity, 2=magnetic touch; a new request restarts motion.
    double target_force = 3.0;  // N, live request for magnetic mode.
    std::uint64_t motion_sequence = 0;  // Changes only for mode/restart, not a slider edit.
};

struct ShmCommand {
    std::uint64_t sequence = 0;  // Controller's own publication counter.
    std::uint64_t published_ns = 0;
    std::uint64_t state_sequence = 0, state_published_ns = 0;
    std::uint64_t request_sequence = 0;
    double control_time = 0;  // Controller's own monotonic elapsed time, seconds.
    int mode = 0;
    bool holding = false;
    MagneticTelemetry magnetic{};
    std::array<double, 3> torque{}, reference{};
};

struct ShmSample {
    std::uint64_t sequence = 0;
    std::uint64_t published_ns = 0;
    std::uint64_t run_id = 0;
    std::uint64_t state_sequence = 0, command_sequence = 0;
    std::uint64_t state_published_ns = 0, command_published_ns = 0;
    double time = 0, control_time = 0;
    int mode = 0;
    std::array<double, 3> position{}, reference{}, torque{};
    MagneticTelemetry magnetic{};
    std::array<double, 3> toe_position{}, contact_force{}, contact_torque{},
        object_position{};  // GT ONLY; never in ShmState.
};

struct ShmJoystick {
    std::uint64_t sequence = 0;
    std::uint64_t published_ns = 0;
    std::array<double, 3> axes{};  // Original joystick mapping: forward, lateral, yaw.
    std::array<bool, 11> buttons{};
};

struct ShmData {
    pthread_mutex_t mutex;
    pthread_cond_t changed;

    bool stopping = false, failed = false, simulation_finished = false;
    std::uint64_t request_sequence = 0;
    std::uint64_t state_sequence = 0, command_sequence = 0, sample_sequence = 0,
                  joystick_sequence = 0;

    // Writers fill the other slot, then publish its sequence under the shared mutex.
    // Readers copy the selected slot under the same mutex, so an old slot cannot
    // be overwritten while a slower reader is still copying it.
    std::array<ShmState, 2> states{};
    std::array<ShmControlRequest, 2> requests{};
    std::array<ShmCommand, 2> commands{};
    std::array<ShmSample, 2> samples{};
    std::array<ShmJoystick, 2> joystick{};
};
