#include "simulation/interaction.hpp"
#include <algorithm>
#include <iostream>

namespace qhrr05 {
SimulationInteraction::SimulationInteraction(
    const std::string& session, const mjModel* model, mjData* data)
    : channel_(session), model_(model), data_(data) {
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&camera_scene_);
    // mjv_defaultScene only zeroes storage. The coordinate conversions used by
    // initPerturb require the identity model transform, even without GL resources.
    camera_scene_.scale = 1;
    camera_scene_.rotate[0] = 1;
}

void SimulationInteraction::Release() {
    if (previous_body_ >= 0)
        mju_zero(data_->xfrc_applied + 6 * previous_body_, 6);
    previous_body_ = -1;
    perturb_.active = 0;
    owner_ = 0;
}

SimulationInteraction::Update SimulationInteraction::Poll(bool paused) {
    Update result;
    result.paused = paused;
    for (const auto& packet : channel_.Poll()) {
        sequence_ = packet.sequence;
        const auto& request = packet.command;
        if (request.action == SimulationAction::Pause) {
            pause_ = packet.sequence;
            result.paused = request.pause != 0;
            Release();
            target_.active = 0;
            std::cout << "[simulation-input] " << (result.paused ? "PAUSED" : "RUNNING")
                      << std::endl;
        } else if (request.action == SimulationAction::Reset) {
            result.reset = true;
            Release();
            target_.active = 0;
            std::cout << "[simulation-input] RESET" << std::endl;
        } else if (request.action == SimulationAction::Step) {
            step_ = packet.sequence;
            result.step = result.paused;
            std::cout << "[simulation-input] SINGLE STEP" << std::endl;
        } else if (request.action == SimulationAction::Perturb) {
            target_ = request.perturb;
            expires_ns_ = packet.expires_ns;
        } else {
            throw std::runtime_error("simulation IPC received unsupported approved command");
        }
    }
    return result;
}

void SimulationInteraction::Apply(bool paused) {
    const auto& target = target_;
    if (!target.active || shm_utils::MonotonicNs() >= expires_ns_) {
        if (perturb_.active) {
            std::cout << "[simulation-input] perturb RELEASE"
                      << (target.active ? " (lease expired)" : "") << std::endl;
            Release();
        }
        return;
    }
    const int body = mj_name2id(model_, mjOBJ_BODY, target.body.data());
    if (body <= 0)
        throw std::runtime_error("approved perturbation body is missing from physics model");
    if (!perturb_.active || owner_ != target.owner || perturb_.select != body) {
        Release();
        perturb_.select = body;
        std::copy(target.local_position.begin(), target.local_position.end(), perturb_.localpos);
        mjvCamera camera;
        mjv_defaultFreeCamera(model_, &camera);
        mjv_updateCamera(model_, data_, &camera, &camera_scene_);
        mjv_initPerturb(model_, data_, &camera_scene_, &perturb_);
        owner_ = target.owner;
        std::cout << "[simulation-input] perturb " << target.body.data()
                  << " active=" << target.active << std::endl;
    }
    perturb_.active = target.active;
    std::copy(target.reference_position.begin(), target.reference_position.end(), perturb_.refpos);
    std::copy(
        target.reference_selection.begin(), target.reference_selection.end(), perturb_.refselpos);
    std::copy(
        target.reference_quaternion.begin(), target.reference_quaternion.end(), perturb_.refquat);
    std::copy(target.local_position.begin(), target.local_position.end(), perturb_.localpos);
    mju_zero(data_->xfrc_applied + 6 * body, 6);
    mjv_applyPerturbPose(model_, data_, &perturb_, paused);
    if (!paused)
        mjv_applyPerturbForce(model_, data_, &perturb_);
    previous_body_ = body;
}

void SimulationInteraction::Publish(SimulationData& state, bool paused) const {
    state.paused = paused;
    state.applied_input_sequence = sequence_;
    state.applied_pause_sequence = pause_;
    state.applied_step_sequence = step_;
    state.perturb_active = perturb_.active;
    state.perturb_owner = owner_;
    state.perturb_wrench.fill(0);
    if (previous_body_ >= 0)
        std::copy_n(data_->xfrc_applied + 6 * previous_body_, 6, state.perturb_wrench.begin());
}
}  // namespace qhrr05
