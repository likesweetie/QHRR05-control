#pragma once

#include <array>
#include <cstdint>

// Shared control values in physical units; packet encoding belongs to the HAL codec.
struct MitCommand {
    double position = 0;  // rad, output-shaft relative coordinate
    double velocity = 0;  // rad/s
    double kp = 0;  // Nm/rad
    double kd = 0;  // Nm*s/rad
    double tau_ff = 0;  // Nm; this platform accepts zero only.
};

struct MotorFeedback {
    double position = 0;
    double velocity = 0;
    double current = 0;
    int temperature = 0;
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
