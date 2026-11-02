#include "motion_controllers/two_order_anticogging.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
double WrapPhase(double phase) {
    constexpr double turn = 6.2831853071795864769;
    double result = std::fmod(phase, turn);
    if (result < 0) result += turn;
    return result == 0 || result >= turn ? 0 : result;
}
}  // namespace
TwoOrderAnticogging::TwoOrderAnticogging(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const auto settings = config["anticogging"];
    if (!settings || !settings["enabled"].as<bool>())
        return;
    enabled_ = true;
    const auto motor_path = (std::filesystem::path(config_path).parent_path() /
        settings["motor_config"].as<std::string>()).lexically_normal();
    const auto motor = YAML::LoadFile(motor_path.string())["mit"];
    gear_ratio_ = motor["gear_ratio"].as<double>();
    kt_output_ = motor["kt_output_nm_a"].as<double>();
    if (motor["torque_max_nm"]) {
        motor_torque_limit_nm_ = motor["torque_max_nm"].as<double>();
        if (!std::isfinite(motor_torque_limit_nm_) || motor_torque_limit_nm_ <= 0)
            throw std::runtime_error("anticogging motor torque_max_nm must be positive and finite");
    }
    if (settings["amplitude_slew_nm_s"])
        amplitude_slew_nm_s_ = settings["amplitude_slew_nm_s"].as<double>();
    if (!std::isfinite(amplitude_slew_nm_s_) || amplitude_slew_nm_s_ <= 0)
        throw std::runtime_error("anticogging.amplitude_slew_nm_s must be positive and finite");
    if (settings["phase_slew_rad_s"])
        phase_slew_rad_s_ = settings["phase_slew_rad_s"].as<double>();
    if (settings["scale"])
        scale_ = settings["scale"].as<double>();
    if (settings["max_motor_comp_nm"])
        max_motor_comp_nm_ = settings["max_motor_comp_nm"].as<double>();
    if (settings["max_comp_current_a"])
        max_comp_current_a_ = settings["max_comp_current_a"].as<double>();
    const auto read_axes = [&](const char* key, double Axis::* member) {
        const auto value = settings[key];
        if (!value)
            return;
        const auto values = value.IsScalar() ? std::vector<double>(3, value.as<double>())
                                             : value.as<std::vector<double>>();
        if (values.size() != 3)
            throw std::runtime_error(std::string("anticogging.") + key + " needs HR/HP/KP values");
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(values[i]))
                throw std::runtime_error(std::string("anticogging.") + key + " must be finite");
            axes_[i].*member = values[i];
        }
    };
    read_axes("motor_zero_rad", &Axis::motor_zero_rad);
    read_axes("amp21_motor_nm", &Axis::amp21_motor_nm);
    read_axes("phase21_rad", &Axis::phase21_rad);
    read_axes("amp36_motor_nm", &Axis::amp36_motor_nm);
    read_axes("phase36_rad", &Axis::phase36_rad);
    if (!std::isfinite(phase_slew_rad_s_) || phase_slew_rad_s_ <= 0)
        throw std::runtime_error("anticogging.phase_slew_rad_s must be positive and finite");
    for (int i = 0; i < 3; ++i) {
        target_amp21_[i] = axes_[i].amp21_motor_nm;
        target_amp36_[i] = axes_[i].amp36_motor_nm;
        target_phase21_[i] = axes_[i].phase21_rad = WrapPhase(axes_[i].phase21_rad);
        target_phase36_[i] = axes_[i].phase36_rad = WrapPhase(axes_[i].phase36_rad);
    }
    if (!std::isfinite(gear_ratio_) || gear_ratio_ <= 0 || !std::isfinite(kt_output_) ||
        kt_output_ <= 0 || !std::isfinite(kt_output_ / gear_ratio_) || kt_output_ / gear_ratio_ <= 0 ||
        !std::isfinite(scale_) || scale_ < 0 || scale_ > 1 ||
        !std::isfinite(max_motor_comp_nm_) || max_motor_comp_nm_ < 0 ||
        !std::isfinite(max_comp_current_a_) || max_comp_current_a_ < 0 ||
        !std::isfinite(max_motor_comp_nm_ * gear_ratio_) ||
        !std::isfinite(max_comp_current_a_ * kt_output_))
        throw std::runtime_error("invalid anticogging gear ratio, Kt, scale or compensation limits");
    for (const auto& a : axes_)
        if (a.amp21_motor_nm < 0 || a.amp36_motor_nm < 0 ||
            !std::isfinite(a.amp21_motor_nm + a.amp36_motor_nm))
            throw std::runtime_error("anticogging amplitudes must be finite and nonnegative");
    std::cout << "[anticogging] ON orders=21,36 motor_config=" << motor_path.string()
              << " GR=" << gear_ratio_ << " Kt_out=" << kt_output_ << " scale=" << scale_ << '\n';
}

void TwoOrderAnticogging::setPhaseTarget(int axis, int order, double phase_rad) {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36) || !std::isfinite(phase_rad))
        throw std::runtime_error("invalid anticogging phase target");
    if (!enabled_)
        throw std::runtime_error("anticogging is disabled in control YAML");
    // Keep targets in [0, 2*pi); updates still follow the shorter arc.
    (order == 21 ? target_phase21_ : target_phase36_)[axis] =
        WrapPhase(phase_rad);
}

double TwoOrderAnticogging::phase(int axis, int order, bool target) const {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36))
        throw std::runtime_error("invalid anticogging phase axis/order");
    if (target)
        return (order == 21 ? target_phase21_ : target_phase36_)[axis];
    return order == 21 ? axes_[axis].phase21_rad : axes_[axis].phase36_rad;
}

void TwoOrderAnticogging::setAmplitudeTarget(int axis, int order, double motor_nm) {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36) ||
        !std::isfinite(motor_nm) || motor_nm < 0)
        throw std::runtime_error("invalid anticogging amplitude target: require nonnegative motor Nm");
    if (!enabled_)
        throw std::runtime_error("anticogging is disabled in control YAML");
    const double other = amplitude(axis, order == 21 ? 36 : 21, true);
    if (!std::isfinite(motor_nm + other))
        throw std::runtime_error("anticogging amplitude sum must be finite");
    (order == 21 ? target_amp21_ : target_amp36_)[axis] = motor_nm;
}

double TwoOrderAnticogging::amplitude(int axis, int order, bool target) const {
    if (axis < 0 || axis >= 3 || (order != 21 && order != 36))
        throw std::runtime_error("invalid anticogging amplitude axis/order");
    if (target) return (order == 21 ? target_amp21_ : target_amp36_)[axis];
    return order == 21 ? axes_[axis].amp21_motor_nm : axes_[axis].amp36_motor_nm;
}

void TwoOrderAnticogging::update(double elapsed_sec) {
    if (!enabled_ || !std::isfinite(elapsed_sec) || elapsed_sec <= 0)
        return;
    // A delayed loop must not catch up with a single large phase/torque jump.
    const double maximum_step = phase_slew_rad_s_ * std::min(elapsed_sec, .01);
    constexpr double turn = 6.2831853071795864769;
    for (int i = 0; i < 3; ++i) {
        for (int order : {21, 36}) {
            double& applied = order == 21 ? axes_[i].phase21_rad : axes_[i].phase36_rad;
            const double delta = std::remainder(phase(i, order, true) - applied, turn);
            applied = WrapPhase(applied + std::clamp(delta, -maximum_step, maximum_step));
            double& amp = order == 21 ? axes_[i].amp21_motor_nm : axes_[i].amp36_motor_nm;
            const double amp_step = amplitude_slew_nm_s_ * std::min(elapsed_sec, .01);
            amp += std::clamp(amplitude(i, order, true) - amp, -amp_step, amp_step);
        }
    }
}

double TwoOrderAnticogging::motorDisturbanceNm(int axis, double joint_angle_rad) const {
    const auto& a = axes_.at(static_cast<std::size_t>(axis));
    constexpr double turn = 6.2831853071795864769;
    const double angle = gear_ratio_ * joint_angle_rad + a.motor_zero_rad;
    if (!std::isfinite(angle))
        throw std::runtime_error("non-finite anticogging motor angle");
    const double theta = std::remainder(angle, turn);
    return a.amp21_motor_nm * std::sin(21 * theta + std::remainder(a.phase21_rad, turn)) +
           a.amp36_motor_nm * std::sin(36 * theta + std::remainder(a.phase36_rad, turn));
}

double TwoOrderAnticogging::compensationIqA(int axis, double joint_angle_rad, unsigned* clipping) const {
    if (clipping) *clipping = 0;
    if (!enabled_ || scale_ == 0 || !std::isfinite(joint_angle_rad))
        return 0;
    const double requested = -motorDisturbanceNm(axis, joint_angle_rad);
    if (clipping && std::abs(requested) > max_motor_comp_nm_) *clipping |= MotorTorque;
    const double opposing_torque = std::clamp(requested, -max_motor_comp_nm_, max_motor_comp_nm_);
    const double iq = opposing_torque / (kt_output_ / gear_ratio_);
    if (clipping && std::abs(iq) > max_comp_current_a_) *clipping |= MotorCurrent;
    return scale_ * std::clamp(iq, -max_comp_current_a_, max_comp_current_a_);
}

double TwoOrderAnticogging::compensationOutputNm(int axis, double joint_angle_rad, unsigned* clipping) const {
    return kt_output_ * compensationIqA(axis, joint_angle_rad, clipping);
}
}  // namespace qhrr05
