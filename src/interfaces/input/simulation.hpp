#pragma once
#include "ipc/simulation/protocol.hpp"

namespace qhrr05 {
void ValidateSimulationInput(const SimulationCommand& command,
    const SimulationData& state,
    const SimulationApproval& previous,
    std::uint64_t now);
}
