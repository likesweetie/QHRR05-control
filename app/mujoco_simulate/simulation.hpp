#pragma once

#include <array>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model);
void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data);
void ExchangeControl(int fd, const mjModel* model, mjData* data, std::array<double, 3>& reference);
}  // namespace qhrr05
