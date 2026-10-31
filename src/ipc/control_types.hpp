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

// Channel meaning is stable across physical and emulated devices. Origin is
// diagnostic metadata and is never an input to a motion controller.
enum class ObservationOrigin : int { Device, EmulatedDevice, DirectDebug };

inline const char* ObservationOriginName(ObservationOrigin origin) {
    switch (origin) {
    case ObservationOrigin::Device:
        return "device";
    case ObservationOrigin::EmulatedDevice:
        return "emulated-device";
    case ObservationOrigin::DirectDebug:
        return "direct-debug";
    }
    throw std::invalid_argument("invalid observation origin");
}

enum MotorSignal : std::uint32_t {
    EncoderPosition = 1,
    JointVelocity = 2,
    MotorCurrent = 4,
    MotorTemperature = 8,
    ConvertedTorque = 16
};

inline constexpr std::uint32_t kCanSignals =
    EncoderPosition | JointVelocity | MotorCurrent | MotorTemperature | ConvertedTorque;
inline constexpr std::uint32_t kDebugSignals = EncoderPosition | JointVelocity | ConvertedTorque;
enum class DriveState : int { Disarmed, Active, Fault, Armed };
enum class DriveAction : int { None, Arm, Release, ClearFault, EnableMotor, DisableMotor };
enum class TransportFault : int {
    None,
    CommandTimeout,
    FeedbackTimeout,
    EnableTimeout,
    InvalidCommand
};

inline const char* DriveStateName(DriveState state) {
    switch (state) {
    case DriveState::Disarmed:
        return "DISARMED";
    case DriveState::Active:
        return "ACTIVE";
    case DriveState::Fault:
        return "FAULT";
    case DriveState::Armed:
        return "ARMED";
    }
    throw std::invalid_argument("invalid drive state");
}

inline const char* TransportFaultName(TransportFault fault) {
    switch (fault) {
    case TransportFault::None:
        return "none";
    case TransportFault::CommandTimeout:
        return "controller command expired";
    case TransportFault::FeedbackTimeout:
        return "motor feedback expired";
    case TransportFault::EnableTimeout:
        return "motor enable/feedback timeout";
    case TransportFault::InvalidCommand:
        return "invalid motor command";
    }
    throw std::invalid_argument("invalid transport fault");
}

enum class ZeroSetPhase : int { Idle, Waiting, Seeking, Backoff, Complete, Cancelled, Fault };
enum class ZeroSetFailure : int {
    None,
    Timeout,
    TravelLimit,
    FeedbackStale,
    InvalidFeedback,
    ExternalContact
};

inline const char* ZeroSetAxisName(int axis) {
    static const char* names[] = {"hip", "thigh", "calf"};
    return axis >= 0 && axis < 3 ? names[axis] : "all";
}

inline const char* ZeroSetPhaseName(ZeroSetPhase phase) {
    switch (phase) {
    case ZeroSetPhase::Idle:
        return "IDLE";
    case ZeroSetPhase::Waiting:
        return "WAITING FOR MOTOR FEEDBACK";
    case ZeroSetPhase::Seeking:
        return "SEEKING STOP";
    case ZeroSetPhase::Backoff:
        return "BACKOFF";
    case ZeroSetPhase::Complete:
        return "COMPLETE";
    case ZeroSetPhase::Cancelled:
        return "CANCELLED";
    case ZeroSetPhase::Fault:
        return "FAULT";
    }
    throw std::invalid_argument("invalid zero-set phase");
}

inline const char* ZeroSetFailureName(ZeroSetFailure reason) {
    switch (reason) {
    case ZeroSetFailure::None:
        return "none";
    case ZeroSetFailure::Timeout:
        return "axis timeout";
    case ZeroSetFailure::TravelLimit:
        return "maximum travel exceeded";
    case ZeroSetFailure::FeedbackStale:
        return "motor feedback stopped";
    case ZeroSetFailure::InvalidFeedback:
        return "non-finite motor feedback";
    case ZeroSetFailure::ExternalContact:
        return "toe touches the scene object; clear the path and retry";
    }
    throw std::invalid_argument("invalid zero-set failure");
}

struct ZeroSetTelemetry {
    ZeroSetPhase phase = ZeroSetPhase::Idle;
    ZeroSetFailure failure = ZeroSetFailure::None;
    int axis = 0;
    bool required = false;
    std::array<bool, 3> calibrated{};
    std::array<double, 3> offset{};
    double travel_rad = 0, elapsed_sec = 0;
};

enum class TrackingPlane : int { XY, XZ, YZ };

enum class TrackingMethod : int { InverseDynamics, CTC, Impedance };

enum class TrackingILCState : int {
    Off,
    Warmup,
    Collecting,
    Active,
    SkippedSaturation,
    SkippedSamples,
    IKHold
};

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
