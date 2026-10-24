#pragma once

#include <array>

#include <string>

namespace qhrr05 {
struct HomingReference {
    std::array<double, 3> position{}, velocity{};
    bool holding{false};
};

class HomingController {
public:
    explicit HomingController(const std::string& config_path);
    void Start(double time, const double position[3]);
    HomingReference Reference(double sim_time) const;
    void Compute(double time, const double position[3], const double velocity[3],
                 double torque[3], double reference[3]) const;
private:
    std::array<double, 3> kp_, kd_, initial_, home_;
    double duration_;
    double start_time_ = 0;
};
}  // namespace qhrr05
