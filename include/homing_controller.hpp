#pragma once

#include "qhrr05_platform.hpp"
#include <mujoco/mujoco.h>

#include <array>
#include <string>

namespace qhrr05 {
using JointValues = std::array<double, kJointCount>;

struct ControlConfig {
    JointValues kp;
    JointValues kd;
    JointValues initial_position;
    JointValues home_position;
    double duration_sec;
    bool bias_compensation;
    double history_sec;
    double sample_hz;
};

ControlConfig LoadControlConfig(const std::string& path);
void ValidateControlConfig(const ControlConfig& config, const mjModel* model);

struct HomingReference {
    JointValues position{};
    JointValues velocity{};
    bool holding{false};
};

class HomingController {
public:
    explicit HomingController(ControlConfig config);
    void Reset(const mjModel* model, mjData* data);
    HomingReference Reference(double sim_time) const;
    HomingReference Apply(const mjModel* model, mjData* data) const;
    const ControlConfig& config() const { return config_; }
private:
    ControlConfig config_;
};
}  // namespace qhrr05
