#pragma once

#include <array>
#include "shm_utils.hpp"

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model);
void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data);
ShmCommand UpdateControlIO(ShmData& shm,
    const mjModel* model,
    mjData* data,
    mjData* gravity_data,
    std::array<double, 3>& reference,
    const std::array<double, 3>& acceleration = {},
    std::uint64_t measurement_sequence = 0);
std::array<double, 6> ReadContactGroundTruth(const mjModel* model, const mjData* data);
}  // namespace qhrr05
