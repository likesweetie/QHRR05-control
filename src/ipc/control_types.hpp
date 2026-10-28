#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

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

enum class TrackingPlane : int { XY, XZ, YZ };

enum class TrackingMethod : int { InverseDynamics, CTC, Impedance };

enum class TrackingILCState : int { Off, Warmup, Collecting, Active, SkippedSaturation, SkippedSamples, IKHold };

inline const char* TrackingILCStateName(TrackingILCState state) {
    switch (state) {
    case TrackingILCState::Off:
        return "OFF";
    case TrackingILCState::Warmup:
        return "WARMUP";
    case TrackingILCState::Collecting:
        return "COLLECTING";
    case TrackingILCState::Active:
        return "ACTIVE";
    case TrackingILCState::SkippedSaturation:
        return "SKIPPED: TORQUE SATURATION";
    case TrackingILCState::SkippedSamples:
        return "SKIPPED: INCOMPLETE PHASE SAMPLES";
    case TrackingILCState::IKHold:
        return "IK HOLD";
    }

    throw std::invalid_argument("invalid tracking ILC state");
}

inline const char* TrackingMethodName(TrackingMethod method) {
    switch (method) {
    case TrackingMethod::InverseDynamics:
        return "inverse_dynamics";
    case TrackingMethod::CTC:
        return "CTC";
    case TrackingMethod::Impedance:
        return "impedance";
    }

    throw std::invalid_argument("invalid tracking method");
}

inline const char* TrackingPlaneName(TrackingPlane plane) {
    switch (plane) {
    case TrackingPlane::XY:
        return "XY";
    case TrackingPlane::XZ:
        return "XZ";
    case TrackingPlane::YZ:
        return "YZ";
    }

    throw std::invalid_argument("invalid tracking plane");
}

inline std::array<int, 2> TrackingPlaneAxes(TrackingPlane plane) {
    switch (plane) {
    case TrackingPlane::XY:
        return {0, 1};
    case TrackingPlane::XZ:
        return {0, 2};
    case TrackingPlane::YZ:
        return {1, 2};
    }

    throw std::invalid_argument("invalid tracking plane");
}

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
