#pragma once

#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include <mujoco/mujoco.h>

namespace qhrr05 {
// Physics-thread-only adapter. Uses the same MuJoCo perturbation functions as Simulate.
class SimulationInteraction {
public:
    struct Update {
        bool paused = false, reset = false, step = false;
    };

    SimulationInteraction(const std::string& session, const mjModel* model, mjData* data);
    Update Poll(bool paused);
    void Apply(bool paused);
    void Publish(SimulationData& state, bool paused) const;

    void PublishState(SimulationData state) {
        channel_.Publish(state);
    }

    void Finish() {
        channel_.Finish();
    }

private:
    void Release();
    simulation_ipc::Server channel_;
    const mjModel* model_;
    mjData* data_;
    PerturbTarget target_{};
    std::uint64_t expires_ns_ = 0;
    mjvPerturb perturb_{};
    mjvScene camera_scene_{};
    std::uint64_t sequence_ = 0, pause_ = 0, reset_ = 0, step_ = 0;
    std::uint64_t owner_ = 0;
    int previous_body_ = -1;
};
}  // namespace qhrr05
