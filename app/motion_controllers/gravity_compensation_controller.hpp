#pragma once

#include <array>
#include <string>

namespace qhrr05 {
class GravityCompensationController {
public:
    explicit GravityCompensationController(const std::string& config_path);
    void Start(const double position[3]);
    void Compute(const double position[3], const double velocity[3], const double gravity[3],
                 double torque[3], double reference[3]) const;
private:
    std::array<double, 3> kp_motor_, kd_motor_, task_stiffness_, desired_{};
    double alpha_;
};
}  // namespace qhrr05
