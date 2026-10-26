#pragma once

#include <array>
#include <cstdint>

namespace openrobot {
struct PositionTarget {
    bool enabled = false;
    double position = 0;
    double velocity = 0;
    double kp = 0;
    double kd = 0;
};

struct MitProfile {
    double velocity_max = 45;
    double torque_max = 53;
    double kt_output = 1.44;
    double gear_ratio = 9;
    unsigned kd_scale = 1;
};

struct Feedback {
    double position = 0;
    double velocity = 0;
    double current = 0;
    int temperature = 0;
};

using Payload = std::array<std::uint8_t, 8>;

// OpenRobot v17: command byte + p16/v12/kp12/kd8/tff8. Torque FF is fixed to 0.
Payload EncodePosition(const PositionTarget& target, const MitProfile& profile);
PositionTarget DecodePosition(const Payload& bytes, const MitProfile& profile);
Payload EncodeFeedback(const Feedback& feedback);
Payload EncodeProfile(const MitProfile& profile);
Feedback DecodeFeedback(const Payload& bytes);
void CheckProfile(const Payload& bytes, const MitProfile& expected);
}  // namespace openrobot
