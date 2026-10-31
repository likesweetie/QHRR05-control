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

void SafetyLayer::Validate(const ControlData& input, bool stopping, bool failed) const {
    if (stopping || failed)
        throw std::runtime_error("safety rejected request: session stopped or failed");

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

void SafetyLayer::ValidateSimulation(const SimulationCommand& command,
    const SimulationData& state,
    const SimulationApproval& previous,
    std::uint64_t now) const {
    if (!state.published_ns)
        throw std::runtime_error("safety rejected simulation request: simulation is not ready");
    if (command.action == SimulationAction::Pause) {
        if (command.pause < 0 || command.pause > 2)
            throw std::runtime_error("safety rejected simulation request: invalid pause value");
        return;
    }
    if (command.action == SimulationAction::Reset)
        return;
    if (command.action == SimulationAction::Step) {
        const bool paused =
            previous.pause_sequence > state.applied_pause_sequence ? previous.paused : state.paused;
        if (!paused)
            throw std::runtime_error("single step requires a paused simulation");
        return;
    }
    if (command.action != SimulationAction::Perturb)
        throw std::runtime_error("safety rejected simulation request: invalid operation");
    const auto& p = command.perturb;
    if (!p.owner || p.active < 0 || p.active > 2)
        throw std::runtime_error("safety rejected perturbation owner or action");
    const char* bodies[] = {"base", "FL_hip", "FL_thigh", "FL_calf", "touch_object"};
    const bool named = p.body.back() == 0 &&
                       std::any_of(std::begin(bodies), std::end(bodies), [&](const char* name) {
                           return std::strcmp(name, p.body.data()) == 0;
                       });
    if (!named)
        throw std::runtime_error("safety rejected perturbation: unknown QHRR body");
    for (const auto* values : {&p.reference_position, &p.reference_selection, &p.local_position})
        for (double value : *values)
            if (!std::isfinite(value))
                throw std::runtime_error("safety rejected perturbation: non-finite position");
    double norm = 0;
    for (double value : p.reference_quaternion) {
        if (!std::isfinite(value))
            throw std::runtime_error("safety rejected perturbation: non-finite quaternion");
        norm += value * value;
    }
    if (std::abs(norm - 1) > .001)
        throw std::runtime_error("safety rejected perturbation: quaternion must be unit length");
    if (previous.perturb.active && previous.perturb_expires_ns > now &&
        previous.perturb.owner != p.owner)
        throw std::runtime_error("perturbation is owned by another viewer; release it first");
}
}  // namespace qhrr05
