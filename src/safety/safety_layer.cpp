#include "safety/safety_layer.hpp"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
SafetyLayer::SafetyLayer(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    maximum_force_ = config["magnetic_touch"]["maximum_force_n"].as<double>();

    if (!std::isfinite(maximum_force_) || maximum_force_ <= 0)
        throw std::runtime_error("invalid configured safety force limit");
}

void SafetyLayer::ValidateAnticoggingPhase(int axis, int order, double delta) const {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36) ||
        !std::isfinite(delta) ||
        (std::abs(std::abs(delta) - .1) > 1e-12 &&
            std::abs(std::abs(delta) - .25) > 1e-12 && std::abs(std::abs(delta) - .5) > 1e-12))
        throw std::runtime_error("safety rejected anticogging phase: expected axis 0..2, order 21/36 and step +/-0.1, +/-0.25 or +/-0.5 rad");
}

void SafetyLayer::ValidateAnticoggingAmplitude(int axis, int order, double delta) const {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36) ||
        !std::isfinite(delta) || std::abs(std::abs(delta) - .01) > 1e-12)
        throw std::runtime_error("safety rejected anticogging amplitude: expected axis 0..2, order 21/36 and step +/-0.01 motor Nm");
}

void SafetyLayer::ValidateRmdDampingConstant(double value) const {
    if (!std::isfinite(value) || value < 0 || value > 100 || value != std::floor(value))
        throw std::runtime_error("RMD damping constant must be an integer in [0, 100]");
}

void SafetyLayer::Validate(const ControlData& input, bool stopping, bool failed) const {
    if (stopping || failed)
        throw std::runtime_error("safety rejected request: session stopped or failed");

    if (int(input.requested_drive_action) < 0 || int(input.requested_drive_action) > 5)
        throw std::runtime_error("safety rejected request: invalid drive action");
    if ((input.requested_drive_action == DriveAction::EnableMotor ||
            input.requested_drive_action == DriveAction::DisableMotor) &&
        (input.requested_motor_axis < -1 || input.requested_motor_axis >= 3))
        throw std::runtime_error("safety rejected request: invalid motor axis");

    if (input.requested_mode < 0 || input.requested_mode > 6)
        throw std::runtime_error("safety rejected request: invalid mode");

    if (int(input.requested_tracking_plane) < 0 || int(input.requested_tracking_plane) > 2)
        throw std::runtime_error("safety rejected request: invalid tracking plane");

    if (!std::isfinite(input.target_force) || input.target_force < 0 ||
        input.target_force > maximum_force_)
        throw std::runtime_error("safety rejected request: force outside configured limits");

    for (double axis : input.joystick_axes)
        if (!std::isfinite(axis) || axis < -1 || axis > 1)
            throw std::runtime_error("safety rejected request: invalid joystick axis");

    for (double trigger : {input.joystick_l2, input.joystick_r2})
        if (!std::isfinite(trigger) || trigger < 0 || trigger > 1)
            throw std::runtime_error("safety rejected request: invalid joystick trigger");

    for (double position : input.requested_ball_position)
        if (!std::isfinite(position))
            throw std::runtime_error("safety rejected request: non-finite ball target");
}

}  // namespace qhrr05
