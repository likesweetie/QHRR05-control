#pragma once

#include "ipc/control_types.hpp"

namespace openrobot {
using Payload = std::array<std::uint8_t, 8>;

// OpenRobot v17: command byte + p16/v12/kp12/kd8/tff8.
Payload EncodeMitCommand(const MitCommand& target, double velocity_max, unsigned kd_scale,
    double torque_max);
MitCommand DecodeMitCommand(const Payload& bytes, double velocity_max, unsigned kd_scale,
    double torque_max);
Payload EncodeFeedback(const MotorFeedback& feedback);
Payload EncodeProfile(
    double velocity_max, double torque_max, double kt_output, double gear_ratio, unsigned kd_scale);
// C5 takes motor-side Kt; callers specify output-side Nm/A like the motor YAML.
Payload EncodeProfileWrite(
    double velocity_max, double torque_max, double kt_output, double gear_ratio, unsigned kd_scale);
MotorFeedback DecodeFeedback(const Payload& bytes);
// RMD A1: output torque -> Iq using verified output-side Kt; fixed +/-33 A range.
// Damping is the firmware's integer level 0..100, not an MIT gain.
Payload EncodeRmdTorque(double torque_nm, double kt_output_nm_a, unsigned damping_constant = 0);
// A1/9C position is a wrapped motor-shaft encoder angle, returned here in
// output-shaft radians. HAL anchors/unwraps it into the existing C3 coordinates.
MotorFeedback DecodeRmdFeedback(const Payload& bytes, double gear_ratio);
Payload EncodeRmdFeedback(const MotorFeedback& feedback, double gear_ratio,
    std::uint8_t opcode = 0xA1);
Payload EncodeZeroPosition(double position_rad);
double DecodeZeroPosition(const Payload& bytes);
void CheckProfile(const Payload& bytes, const Payload& expected);
}  // namespace openrobot
