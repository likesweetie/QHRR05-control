#pragma once

#include "ipc/shared_memory.hpp"
#include <algorithm>
#include <cmath>

namespace qhrr05 {
// No source identity: these are the only acquisition facts needed by control.
struct JointObservation {
    std::array<double, 3> encoder{}, position{}, velocity{}, torque{};
    std::array<bool, 3> coordinate_valid{};
    std::array<std::uint64_t, 3> sequence{}, sample_ns{}, received_ns{};
    bool position_velocity = false, velocity_valid = false, torque_valid = false, ready = false;
};

inline bool HasSignals(const MotorData& data, int axis, std::uint32_t signals) {
    return (data.supported[axis] & data.valid[axis] & signals) == signals &&
           data.received_sequence[axis] != 0;
}

inline JointObservation ReadObservation(const MotorData& data) {
    JointObservation result;
    result.position_velocity = result.velocity_valid = result.torque_valid = result.ready = true;
    result.coordinate_valid = data.coordinate_valid;
    result.sequence = data.received_sequence;
    result.sample_ns = data.measurement_ns;
    result.received_ns = data.received_ns;
    for (int i = 0; i < 3; ++i) {
        result.encoder[i] = data.feedback[i].position;
        result.position[i] = data.position_offset[i] + result.encoder[i];
        result.velocity[i] = data.feedback[i].velocity;
        result.torque[i] = data.motor_torque[i];
        result.velocity_valid &=
            HasSignals(data, i, JointVelocity) && std::isfinite(result.velocity[i]);
        result.position_velocity &= HasSignals(data, i, EncoderPosition | JointVelocity) &&
                                    std::isfinite(result.encoder[i]) &&
                                    std::isfinite(result.position[i]) &&
                                    std::isfinite(result.velocity[i]);
        result.torque_valid &=
            HasSignals(data, i, ConvertedTorque) && std::isfinite(result.torque[i]);
        result.ready &= data.enabled[i];
    }
    return result;
}

inline bool CoordinatesValid(const std::array<bool, 3>& valid) {
    return std::all_of(valid.begin(), valid.end(), [](bool value) {
        return value;
    });
}
}  // namespace qhrr05
