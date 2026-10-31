#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
// Shared configuration only: no controller, model, transport or runtime state.
inline std::array<double, 3> ReadControlTorqueLimits(const YAML::Node& config) {
    for (const char* name : {"homing",
             "gravity_compensation",
             "magnetic_touch",
             "tracking",
             "ball_following",
             "zero_set"}) {
        const auto section = config[name];
        if (section && (section["torque_limit_nm"] || section["output_torque_limit_nm"]))
            throw std::runtime_error(
                std::string(name) + ": move output torque limits to limits.torque_nm");
    }
    if (!config["limits"] || !config["limits"]["torque_nm"])
        throw std::runtime_error("limits.torque_nm is required for every controller");
    const auto value = config["limits"]["torque_nm"];
    std::array<double, 3> limits;
    if (value.IsScalar())
        limits.fill(value.as<double>());
    else {
        const auto values = value.as<std::vector<double>>();
        if (values.size() != 3)
            throw std::runtime_error("limits.torque_nm requires a scalar or three joint values");
        std::copy(values.begin(), values.end(), limits.begin());
    }
    for (double limit : limits)
        if (!std::isfinite(limit) || limit <= 0)
            throw std::runtime_error("limits.torque_nm must be finite and positive");
    return limits;
}
}  // namespace qhrr05
