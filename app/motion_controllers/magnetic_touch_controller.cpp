#include "magnetic_touch_controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    double value = 0;
    for (int i = 0; i < 3; ++i)
        value += a[i] * b[i];
    return value;
}

double Norm(const std::array<double, 3>& v) {
    return std::sqrt(Dot(v, v));
}
}

MagneticTouchController::MagneticTouchController(const std::string& path) : observer_(path) {
    const auto config = YAML::LoadFile(path);
    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto s = config["magnetic_touch"];
    const auto scalar = [&](const char* key) {
        const double value = s[key].as<double>();
        if (!std::isfinite(value) || value < 0)
            throw std::runtime_error(std::string("invalid magnetic parameter: ") + key);
        return value;
    };
    const auto joints = [&](const char* key) {
        std::array<double, 3> result;
        const auto values = s[key].as<std::vector<double>>();
        if (values.size() != 3)
            throw std::runtime_error(std::string("expected three magnetic values: ") + key);
        std::copy(values.begin(), values.end(), result.begin());
        for (double value : result)
            if (!std::isfinite(value))
                throw std::runtime_error("magnetic values must be finite");
        return result;
    };

    damping_ = joints("joint_damping_nm_s_rad");
    torque_limit_ = joints("torque_limit_nm");
    workspace_min_ = joints("workspace_min_m");
    workspace_max_ = joints("workspace_max_m");

    contact_on_ = scalar("contact_on_n");
    contact_cancel_ = scalar("contact_cancel_n");
    contact_off_ = scalar("contact_off_n");
    contact_recover_ = scalar("contact_recover_n");
    debounce_ = scalar("contact_debounce_sec");
    release_delay_ = scalar("release_sec");
    ramp_time_ = scalar("force_ramp_sec");

    maximum_force_ = scalar("maximum_force_n");
    force_gain_ = scalar("force_gain");
    admittance_ = scalar("admittance_m_per_ns");
    stiffness_ = scalar("tracking_stiffness_n_m");
    cartesian_damping_ = scalar("tracking_damping_ns_m");
    maximum_speed_ = scalar("maximum_velocity_m_s");
    maximum_distance_ = scalar("maximum_displacement_m");
    joint_margin_ = scalar("joint_margin_rad");

    const double target = scalar("target_force_n");
    if (!(contact_off_ < contact_recover_ && contact_recover_ <= contact_cancel_ &&
            contact_cancel_ < contact_on_) ||
        contact_on_ >= maximum_force_ || target > maximum_force_ || ramp_time_ <= 0 ||
        release_delay_ <= 0 || maximum_speed_ <= 0 || maximum_distance_ <= 0)
        throw std::runtime_error("invalid magnetic thresholds, force, time or motion bounds");
    for (int i = 0; i < 3; ++i)
        if (damping_[i] < 0 || torque_limit_[i] <= 0 || workspace_min_[i] >= workspace_max_[i])
            throw std::runtime_error("invalid magnetic joint/workspace bounds");
}

void MagneticTouchController::Transition(MagneticPhase phase, MagneticReason reason, double time) {
    if (telemetry_.phase != phase || telemetry_.reason != reason) {
        ++telemetry_.event_sequence;
        telemetry_.event_time = time;
    }

    telemetry_.phase = phase;
    telemetry_.reason = reason;
    phase_since_ = time;
}

void MagneticTouchController::Release(MagneticReason reason, double time) {
    Transition(MagneticPhase::Free, reason, time);
    telemetry_.force_command = 0;
    telemetry_.displacement = 0;
    ramped_force_ = ramp_to_ = ramp_from_ = 0;
    armed_ = false;
}

void MagneticTouchController::Start(double time, const ShmState& feedback) {
    const auto event = telemetry_.event_sequence + 1;
    telemetry_ = {};
    telemetry_.event_sequence = event;
    telemetry_.event_time = time;
    telemetry_.toe_reference = feedback.toe_position;
    observer_.Reset();
    previous_time_ = phase_since_ = ramp_since_ = time;
    ramp_from_ = ramp_to_ = ramped_force_ = 0;
    armed_ = true;
}

void MagneticTouchController::Compute(double time,
    const ShmState& f,
    double target_force,
    std::array<double, 3>& torque,
    std::array<double, 3>& reference) {
    if (!std::isfinite(time) || time < previous_time_ || !std::isfinite(target_force) ||
        target_force < 0 || target_force > maximum_force_)
        throw std::runtime_error("invalid magnetic time or target force");

    const double dt = time - previous_time_;
    previous_time_ = time;

    observer_.Update(f);
    telemetry_.observer_valid = observer_.valid();
    telemetry_.estimated_force = observer_.force();
    telemetry_.external_torque = observer_.torque();
    telemetry_.minimum_singular_value = observer_.minimumSingularValue();
    telemetry_.target_force = target_force;
    telemetry_.force_command = 0;
    telemetry_.torque_saturated = false;

    reference = f.position;
    for (int i = 0; i < 3; ++i) {
        torque[i] = f.gravity_torque[i] - damping_[i] * f.velocity[i];
        if (!std::isfinite(torque[i]))
            throw std::runtime_error("invalid magnetic joint feedback");
    }

    const auto bounds = [&]() {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(f.toe_position[i]) || f.toe_position[i] < workspace_min_[i] ||
                f.toe_position[i] > workspace_max_[i])
                return MagneticReason::Workspace;
            if (f.joint_max[i] > f.joint_min[i] &&
                (f.position[i] < f.joint_min[i] + joint_margin_ ||
                    f.position[i] > f.joint_max[i] - joint_margin_))
                return MagneticReason::JointLimit;
        }
        return MagneticReason::None;
    };

    MagneticReason invalid =
        !f.measurement_sequence ? MagneticReason::NoMeasurement
                                : (!observer_.valid() ? MagneticReason::ObserverInvalid : bounds());
    if (invalid != MagneticReason::None) {
        Release(invalid, time);
        telemetry_.toe_reference = f.toe_position;
    } else {
        const double magnitude = Norm(telemetry_.estimated_force);
        if (!armed_ && magnitude < contact_cancel_) {
            armed_ = true;
            if (telemetry_.reason == MagneticReason::NoMeasurement ||
                telemetry_.reason == MagneticReason::ObserverInvalid)
                Transition(MagneticPhase::Free, MagneticReason::None, time);
        }

        if (telemetry_.phase == MagneticPhase::Free) {
            telemetry_.toe_reference = f.toe_position;
            telemetry_.estimated_normal_force = magnitude;
            if (armed_ && magnitude > contact_on_)
                Transition(MagneticPhase::Candidate, MagneticReason::None, time);
        } else if (telemetry_.phase == MagneticPhase::Candidate) {
            telemetry_.toe_reference = f.toe_position;
            telemetry_.estimated_normal_force = magnitude;
            if (magnitude < contact_cancel_)
                Transition(MagneticPhase::Free, MagneticReason::None, time);
            else if (time - phase_since_ >= debounce_) {
                for (int i = 0; i < 3; ++i)
                    telemetry_.normal[i] = -telemetry_.estimated_force[i] / magnitude;
                contact_position_ = f.toe_position;
                telemetry_.displacement = 0;
                ramp_from_ = ramped_force_ = 0;
                ramp_to_ = target_force;
                ramp_since_ = time;
                Transition(MagneticPhase::Following, MagneticReason::None, time);
            }
        }

        if (telemetry_.phase == MagneticPhase::Following ||
            telemetry_.phase == MagneticPhase::Releasing) {
            telemetry_.estimated_normal_force = -Dot(telemetry_.normal, telemetry_.estimated_force);
            std::array<double, 3> delta{}, velocity{};
            for (int i = 0; i < 3; ++i) {
                delta[i] = f.toe_position[i] - contact_position_[i];
                for (int j = 0; j < 3; ++j)
                    velocity[i] += f.jacobian[3 * i + j] * f.velocity[j];
            }

            if (Norm(delta) > maximum_distance_)
                Release(MagneticReason::StrokeLimit, time);
            else {
                if (telemetry_.phase == MagneticPhase::Following &&
                    telemetry_.estimated_normal_force < contact_off_) {
                    Transition(MagneticPhase::Releasing, MagneticReason::None, time);
                    ramp_from_ = ramped_force_;
                    ramp_to_ = 0;
                    ramp_since_ = time;
                } else if (telemetry_.phase == MagneticPhase::Releasing) {
                    if (telemetry_.estimated_normal_force > contact_recover_) {
                        Transition(MagneticPhase::Following, MagneticReason::None, time);
                        ramp_from_ = ramped_force_;
                        ramp_to_ = target_force;
                        ramp_since_ = time;
                    } else if (time - phase_since_ >= release_delay_)
                        Release(MagneticReason::ContactLost, time);
                }

                if (telemetry_.phase == MagneticPhase::Following ||
                    telemetry_.phase == MagneticPhase::Releasing) {
                    if (telemetry_.phase == MagneticPhase::Following && target_force != ramp_to_) {
                        ramp_from_ = ramped_force_;
                        ramp_to_ = target_force;
                        ramp_since_ = time;
                    }

                    const double s = std::clamp((time - ramp_since_) / ramp_time_, 0.0, 1.0);
                    ramped_force_ = ramp_from_ + (ramp_to_ - ramp_from_) * s * s * (3 - 2 * s);
                    const double error = ramped_force_ - telemetry_.estimated_normal_force;
                    const double old_displacement = telemetry_.displacement;
                    if (telemetry_.phase == MagneticPhase::Following) {
                        const double speed =
                            std::clamp(admittance_ * error, -maximum_speed_, maximum_speed_);
                        telemetry_.displacement = std::clamp(
                            old_displacement + dt * speed, -maximum_distance_, maximum_distance_);
                    }  // Contact uncertainty immediately freezes the position integrator.
                    for (int i = 0; i < 3; ++i)
                        telemetry_.toe_reference[i] =
                            contact_position_[i] + telemetry_.normal[i] * telemetry_.displacement;

                    std::array<double, 3> tracking{};
                    for (int i = 0; i < 3; ++i)
                        tracking[i] = telemetry_.toe_reference[i] - f.toe_position[i];
                    const double raw_force = ramped_force_ + force_gain_ * error +
                                             stiffness_ * Dot(telemetry_.normal, tracking) -
                                             cartesian_damping_ * Dot(telemetry_.normal, velocity);
                    telemetry_.force_command = std::clamp(raw_force, 0.0, maximum_force_);
                    bool saturated = raw_force != telemetry_.force_command;

                    for (int joint = 0; joint < 3; ++joint) {
                        double contact_torque = 0;
                        for (int axis = 0; axis < 3; ++axis)
                            contact_torque += f.jacobian[3 * axis + joint] *
                                              telemetry_.normal[axis] * telemetry_.force_command;
                        torque[joint] += contact_torque;
                        saturated |= std::abs(torque[joint]) > torque_limit_[joint];
                    }

                    if (saturated) {
                        telemetry_.displacement = old_displacement;  // Anti-windup.
                        for (int i = 0; i < 3; ++i)
                            telemetry_.toe_reference[i] =
                                contact_position_[i] + telemetry_.normal[i] * old_displacement;
                    }
                }
            }
        }
    }

    if (telemetry_.phase == MagneticPhase::Free)
        telemetry_.toe_reference = f.toe_position;
    for (int i = 0; i < 3; ++i) {
        telemetry_.torque_saturated |= std::abs(torque[i]) > torque_limit_[i];
        torque[i] = std::clamp(torque[i], -torque_limit_[i], torque_limit_[i]);
    }
}
}  // namespace qhrr05
