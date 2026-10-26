#include "motor_can/openrobot_protocol.hpp"

#include <cmath>
#include <stdexcept>

namespace openrobot {
namespace {
unsigned Quantize(double value, double minimum, double maximum, unsigned bits) {
    if (!std::isfinite(value) || value < minimum || value > maximum)
        throw std::runtime_error("MIT command is non-finite or outside protocol range");

    return static_cast<unsigned>(
        std::lround((value - minimum) / (maximum - minimum) * ((1U << bits) - 1)));
}

int Signed16(const Payload& bytes, int offset) {
    const unsigned raw = unsigned(bytes[offset]) | (unsigned(bytes[offset + 1]) << 8);

    return raw >= 0x8000 ? int(raw) - 0x10000 : int(raw);
}
}  // namespace

Payload EncodeMitCommand(const MitCommand& target, double velocity_max, unsigned kd_scale) {
    if (!std::isfinite(velocity_max) || velocity_max <= 0 || kd_scale < 1 || kd_scale > 8)
        throw std::runtime_error("invalid MIT encoding profile");

    if (target.tau_ff != 0)
        throw std::runtime_error("MIT command tau_ff must be zero");

    const unsigned p = Quantize(target.position, -12.5, 12.5, 16);
    const unsigned v = Quantize(target.velocity, -velocity_max, velocity_max, 12);
    const unsigned kp = Quantize(target.kp, 0, 500, 12);
    const unsigned kd = Quantize(target.kd, 0, 5.0 * kd_scale, 8);

    // Logical zero in the unsigned symmetric 8-bit torque field. Never raw byte 0
    // (which means -T_MAX). The half-LSB quantization is documented, not hidden.
    return {0xC0,
        std::uint8_t(p >> 8),
        std::uint8_t(p),
        std::uint8_t(v >> 4),
        std::uint8_t((v << 4) | (kp >> 8)),
        std::uint8_t(kp),
        std::uint8_t(kd),
        0x80};
}

MotorFeedback DecodeFeedback(const Payload& bytes) {
    if (bytes[0] != 0xC0)
        throw std::runtime_error("not an OpenRobot MIT feedback frame");

    MotorFeedback result;
    result.temperature = bytes[1] < 128 ? int(bytes[1]) : int(bytes[1]) - 256;
    result.current = Signed16(bytes, 2) * (33.0 / 2048.0);
    result.velocity = Signed16(bytes, 4) * (3.141592653589793 / 180.0);
    result.position = Signed16(bytes, 6) * (12.56 / 32768.0);

    return result;
}

MitCommand DecodeMitCommand(const Payload& bytes, double velocity_max, unsigned kd_scale) {
    if (bytes[0] != 0xC0 || bytes[7] != 0x80)
        throw std::runtime_error("virtual motor requires MIT command with logical zero FF");

    const unsigned p = (unsigned(bytes[1]) << 8) | bytes[2];
    const unsigned v = (unsigned(bytes[3]) << 4) | (bytes[4] >> 4);
    const unsigned kp = (unsigned(bytes[4] & 15) << 8) | bytes[5];

    return {p * (25.0 / 65535.0) - 12.5,
        v * (2 * velocity_max / 4095.0) - velocity_max,
        kp * (500.0 / 4095.0),
        bytes[6] * (5.0 * kd_scale / 255.0),
        0};
}

Payload EncodeFeedback(const MotorFeedback& feedback) {
    Payload bytes{0xC0};
    const auto put = [&](int offset, double value) {
        if (!std::isfinite(value) || value < -32768 || value > 32767)
            throw std::runtime_error("virtual motor feedback exceeds signed 16-bit range");

        const auto raw = static_cast<std::uint16_t>(static_cast<std::int16_t>(std::lround(value)));
        bytes[offset] = raw & 255;
        bytes[offset + 1] = raw >> 8;
    };

    if (feedback.temperature < -128 || feedback.temperature > 127)
        throw std::runtime_error("virtual motor temperature exceeds signed 8-bit range");

    bytes[1] = static_cast<std::uint8_t>(feedback.temperature);
    put(2, feedback.current * (2048.0 / 33.0));
    put(4, feedback.velocity * (180.0 / 3.141592653589793));
    put(6, feedback.position * (32768.0 / 12.56));

    return bytes;
}

Payload EncodeProfile(double velocity_max,
    double torque_max,
    double kt_output,
    double gear_ratio,
    unsigned kd_scale) {
    if (!std::isfinite(velocity_max) || velocity_max < 1 || velocity_max > 255 ||
        velocity_max != std::floor(velocity_max) || !std::isfinite(torque_max) || torque_max < 1 ||
        torque_max > 255 || torque_max != std::floor(torque_max) || !std::isfinite(kt_output) ||
        kt_output <= 0 || kt_output > 65.535 || !std::isfinite(gear_ratio) || gear_ratio <= 0 ||
        gear_ratio > 655.35 || kd_scale < 1 || kd_scale > 8)
        throw std::runtime_error("invalid MIT profile response fields");

    const auto kt = static_cast<unsigned>(std::lround(kt_output * 1000));
    const auto ratio = static_cast<unsigned>(std::lround(gear_ratio * 100));
    Payload result{0xC4,
        std::uint8_t(velocity_max),
        std::uint8_t(torque_max),
        std::uint8_t(kt),
        std::uint8_t(kt >> 8),
        std::uint8_t(ratio),
        std::uint8_t(ratio >> 8),
        std::uint8_t(kd_scale)};

    return result;
}

void CheckProfile(const Payload& bytes, const Payload& expected) {
    if (expected[0] != 0xC4 || bytes != expected)
        throw std::runtime_error(
            "MIT C4 profile mismatch (V_MAX/T_MAX/Kt_out/gear/kd_scale); refusing enable");
}
}  // namespace openrobot
