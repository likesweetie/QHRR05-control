#pragma once

#include <array>

#include <yaml-cpp/yaml.h>

namespace qhrr05 {
struct HomingReference {
    std::array<double, 3> position{}, velocity{};
    bool holding{false};
};

class HomingController {
public:
    explicit HomingController(const YAML::Node& settings);
    HomingReference Reference(double sim_time) const;
    void Compute(double time, const double position[3], const double velocity[3],
                 const double bias[3], double torque[3], double reference[3]) const;
private:
    std::array<double, 3> kp_, kd_, initial_, home_;
    double duration_;
    bool bias_compensation_;
};
}  // namespace qhrr05
