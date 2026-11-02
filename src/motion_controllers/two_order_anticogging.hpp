#pragma once

#include <array>
#include <string>

namespace qhrr05 {
// Feedforward against 21st/36th-order motor-shaft disturbance, referenced to MIT q=0.
class TwoOrderAnticogging {
public:
    explicit TwoOrderAnticogging(const std::string& config_path);
    bool enabled() const { return enabled_; }
    void setPhaseTarget(int axis, int order, double phase_rad);
    void setAmplitudeTarget(int axis, int order, double motor_nm);
    void update(double elapsed_sec);
    double amplitude(int axis, int order, bool target = false) const;
    double amplitudeSlewRate() const { return amplitude_slew_nm_s_; }
    double maxMotorCompNm() const { return max_motor_comp_nm_; }
    double maxCompCurrentA() const { return max_comp_current_a_; }
    double motorTorqueLimitNm() const { return motor_torque_limit_nm_; }
    enum Clipping : unsigned { MotorTorque = 1, MotorCurrent = 2, CombinedTorque = 4, CanTorque = 8 };
    double phase(int axis, int order, bool target = false) const;
    double phaseSlewRate() const { return phase_slew_rad_s_; }
    double motorDisturbanceNm(int axis, double joint_angle_rad) const;
    double compensationIqA(int axis, double joint_angle_rad, unsigned* clipping = nullptr) const;
    double compensationOutputNm(int axis, double joint_angle_rad, unsigned* clipping = nullptr) const;

private:
    struct Axis {
        double motor_zero_rad = 0;
        double amp21_motor_nm = .05, phase21_rad = 0;
        double amp36_motor_nm = .05, phase36_rad = 0;
    };
    bool enabled_ = false;
    double gear_ratio_ = 9, kt_output_ = 1.298;
    double max_motor_comp_nm_ = .10, max_comp_current_a_ = .70, scale_ = .1;
    std::array<Axis, 3> axes_{};
    std::array<double, 3> target_phase21_{}, target_phase36_{};
    double phase_slew_rad_s_ = 5.0;
    std::array<double, 3> target_amp21_{.05, .05, .05}, target_amp36_{.05, .05, .05};
    double amplitude_slew_nm_s_ = .1, motor_torque_limit_nm_ = 0;
};
}  // namespace qhrr05
