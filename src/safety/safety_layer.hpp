#pragma once

#include "ipc/simulation/protocol.hpp"

#include "ipc/shared_memory.hpp"
#include <string>

namespace qhrr05 {
class SafetyLayer {
public:
    explicit SafetyLayer(const std::string& config_path);
    void Validate(const ControlData& input, bool stopping, bool failed) const;
    void ValidateSimulation(const SimulationCommand& command,
        const SimulationData& state,
        const SimulationApproval& previous,
        std::uint64_t now) const;

private:
    double maximum_force_ = 0;
};
}  // namespace qhrr05
