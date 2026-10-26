#pragma once

#include "ipc/control_types.hpp"

namespace openrobot {
using Payload = std::array<std::uint8_t, 8>;

// OpenRobot v17: command byte + p16/v12/kp12/kd8/tff8. Torque FF is fixed to 0.
Payload EncodeMitCommand(const MitCommand& target, double velocity_max, unsigned kd_scale);
MitCommand DecodeMitCommand(const Payload& bytes, double velocity_max, unsigned kd_scale);
Payload EncodeFeedback(const MotorFeedback& feedback);
Payload EncodeProfile(
    double velocity_max, double torque_max, double kt_output, double gear_ratio, unsigned kd_scale);
MotorFeedback DecodeFeedback(const Payload& bytes);
void CheckProfile(const Payload& bytes, const Payload& expected);
}  // namespace openrobot
