#include "motor_can_protocol/openrobot_protocol.hpp"

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>
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

Payload EncodeMitCommand(const MitCommand& target, double velocity_max, unsigned kd_scale,
    double torque_max) {
    if (!std::isfinite(velocity_max) || velocity_max <= 0 || kd_scale < 1 || kd_scale > 8 ||
        !std::isfinite(torque_max) || torque_max <= 0)
        throw std::runtime_error("invalid MIT encoding profile");

    const unsigned p = Quantize(target.position, -12.5, 12.5, 16);
    const unsigned v = Quantize(target.velocity, -velocity_max, velocity_max, 12);
    const unsigned kp = Quantize(target.kp, 0, 500, 12);
    const unsigned kd = Quantize(target.kd, 0, 5.0 * kd_scale, 8);
    const unsigned torque = Quantize(target.tau_ff, -torque_max, torque_max, 8);

    // Symmetric unsigned torque encoding has no exact zero (0 -> 0x80).
    return {0xC0,
        std::uint8_t(p >> 8),
        std::uint8_t(p),
        std::uint8_t(v >> 4),
        std::uint8_t((v << 4) | (kp >> 8)),
        std::uint8_t(kp),
        std::uint8_t(kd),
        std::uint8_t(torque)};
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

Payload EncodeRmdTorque(double torque_nm, double kt_output_nm_a, unsigned damping_constant) {
    if (!std::isfinite(torque_nm) || !std::isfinite(kt_output_nm_a) || kt_output_nm_a <= 0 ||
        damping_constant > 100)
        throw std::runtime_error("invalid RMD torque or output-side Kt");
    const double current = std::clamp(torque_nm / kt_output_nm_a, -33.0, 33.0);
    const auto raw = static_cast<std::uint16_t>(static_cast<std::int16_t>(
        std::lround(current * (2048.0 / 33.0))));
    return {0xA1, 0, 0, 0, std::uint8_t(raw), std::uint8_t(raw >> 8),
        std::uint8_t(damping_constant), 0};
}

MotorFeedback DecodeRmdFeedback(const Payload& bytes, double gear_ratio) {
    if ((bytes[0] != 0xA1 && bytes[0] != 0x9C) ||
        !std::isfinite(gear_ratio) || gear_ratio <= 0)
        throw std::runtime_error("invalid RMD feedback or gear ratio");
    constexpr double pi = 3.14159265358979323846;
    const unsigned encoder = (unsigned(bytes[6]) | (unsigned(bytes[7]) << 8)) & 0x3fff;
    return {encoder * (2 * pi / 16384.0) / gear_ratio,
        Signed16(bytes, 4) * (pi / 180.0) / gear_ratio,
        Signed16(bytes, 2) * (33.0 / 2048.0),
        bytes[1] < 128 ? int(bytes[1]) : int(bytes[1]) - 256};
}

Payload EncodeRmdFeedback(const MotorFeedback& feedback, double gear_ratio, std::uint8_t opcode) {
    if ((opcode != 0xA1 && opcode != 0x9C) || !std::isfinite(gear_ratio) || gear_ratio <= 0 ||
        !std::isfinite(feedback.position) || !std::isfinite(feedback.velocity) ||
        !std::isfinite(feedback.current) || feedback.temperature < -128 || feedback.temperature > 127)
        throw std::runtime_error("invalid virtual RMD feedback");
    constexpr double pi = 3.14159265358979323846;
    Payload bytes{opcode, static_cast<std::uint8_t>(feedback.temperature)};
    const auto put = [&](int offset, double value) {
        if (!std::isfinite(value) || value < -32768 || value > 32767)
            throw std::runtime_error("virtual RMD feedback exceeds signed 16-bit range");
        const auto raw = static_cast<std::uint16_t>(static_cast<std::int16_t>(std::lround(value)));
        bytes[offset] = std::uint8_t(raw);
        bytes[offset + 1] = std::uint8_t(raw >> 8);
    };
    put(2, feedback.current * (2048.0 / 33.0));
    put(4, feedback.velocity * gear_ratio * (180.0 / pi));
    double angle = std::fmod(feedback.position * gear_ratio, 2 * pi);
    if (angle < 0) angle += 2 * pi;
    const unsigned encoder = static_cast<unsigned>(std::lround(angle * (16384.0 / (2 * pi)))) & 0x3fff;
    bytes[6] = std::uint8_t(encoder);
    bytes[7] = std::uint8_t(encoder >> 8);
    return bytes;
}

MitCommand DecodeMitCommand(const Payload& bytes, double velocity_max, unsigned kd_scale,
    double torque_max) {
    if (bytes[0] != 0xC0 || !std::isfinite(velocity_max) || velocity_max <= 0 ||
        kd_scale < 1 || kd_scale > 8 || !std::isfinite(torque_max) || torque_max <= 0)
        throw std::runtime_error("invalid MIT command or decoding profile");

    const unsigned p = (unsigned(bytes[1]) << 8) | bytes[2];
    const unsigned v = (unsigned(bytes[3]) << 4) | (bytes[4] >> 4);
    const unsigned kp = (unsigned(bytes[4] & 15) << 8) | bytes[5];

    return {p * (25.0 / 65535.0) - 12.5,
        v * (2 * velocity_max / 4095.0) - velocity_max,
        kp * (500.0 / 4095.0),
        bytes[6] * (5.0 * kd_scale / 255.0),
        bytes[7] * (2 * torque_max / 255.0) - torque_max};
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

Payload EncodeProfileWrite(double velocity_max,
    double torque_max,
    double kt_output,
    double gear_ratio,
    unsigned kd_scale) {
    auto bytes = EncodeProfile(velocity_max, torque_max, kt_output, gear_ratio, kd_scale);
    const unsigned ratio = unsigned(bytes[5]) | (unsigned(bytes[6]) << 8);
    if (!ratio)
        throw std::runtime_error("C5 gear ratio is below wire resolution");
    const double input = std::round(kt_output * 100000.0 / ratio);
    if (input < 1 || input > 65535 || std::round(input * ratio / 100.0) > 65535)
        throw std::runtime_error("C5 Kt is outside input/output wire range");
    const auto kt = static_cast<unsigned>(input);
    bytes[0] = 0xC5;
    bytes[3] = std::uint8_t(kt);
    bytes[4] = std::uint8_t(kt >> 8);
    return bytes;
}

Payload EncodeZeroPosition(double position_rad) {
    const double centidegrees = position_rad * (18000.0 / 3.141592653589793);
    if (!std::isfinite(centidegrees) || centidegrees < -32768 || centidegrees > 32767)
        throw std::runtime_error("C3 zero position exceeds signed 0.01 degree range");
    const auto raw = static_cast<std::uint16_t>(static_cast<std::int16_t>(std::lround(centidegrees)));
    return {0xC3, 0, 0, 0, 0, 0, std::uint8_t(raw), std::uint8_t(raw >> 8)};
}

double DecodeZeroPosition(const Payload& bytes) {
    if (bytes[0] != 0xC3)
        throw std::runtime_error("not a C3 zero position response");
    return Signed16(bytes, 6) * (3.141592653589793 / 18000.0);
}

void CheckProfile(const Payload& bytes, const Payload& expected) {
    if (expected[0] == 0xC4 && bytes == expected)
        return;

    std::ostringstream message;
    message.imbue(std::locale::classic());
    message << "MIT C4 profile mismatch; refusing enable";
    const auto field = [&](const char* name, unsigned actual, unsigned wanted,
                           unsigned divisor = 1, int precision = 0) {
        message << '\n' << "  " << name << ": actual=" << std::fixed
                << std::setprecision(precision) << double(actual) / divisor
                << " expected=" << double(wanted) / divisor
                << (actual == wanted ? " [OK]" : " [MISMATCH]");
    };
    const auto word = [](const Payload& payload, int offset) {
        return unsigned(payload[offset]) | (unsigned(payload[offset + 1]) << 8);
    };
    field("opcode", bytes[0], expected[0]);
    field("velocity_max_rad_s (V_MAX, rad/s)", bytes[1], expected[1]);
    field("torque_max_nm (T_MAX, Nm)", bytes[2], expected[2]);
    field("kt_output_nm_a (Kt_out, Nm/A)", word(bytes, 3), word(expected, 3), 1000, 3);
    field("gear_ratio", word(bytes, 5), word(expected, 5), 100, 2);
    field("kd_scale", bytes[7], expected[7]);
    const auto raw = [&](const char* name, const Payload& payload) {
        message << '\n' << "  " << name << ':' << std::hex << std::uppercase
                << std::setfill('0');
        for (auto byte : payload)
            message << ' ' << std::setw(2) << unsigned(byte);
    };
    raw("actual bytes", bytes);
    raw("expected bytes", expected);
    throw std::runtime_error(message.str());
}
}  // namespace openrobot
