#include "motion_controllers/magnetic_touch_controller.hpp"
#include "control/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
using numeric::Dot;
using numeric::Norm;

MagneticTouchController::MagneticTouchController(const std::string& path) : observer_(path) {
    const auto config = YAML::LoadFile(path);
    const bool torque_ref_as_position = ReadTorqueRefAsPosition(config);

    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto s = config["magnetic_touch"];
    gravity_only_ = s["gravity_only"].as<bool>();
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

    kp_motor_ = joints("kp_motor");
    kd_motor_ = joints("kd_motor");
    torque_limit_ = ReadControlTorqueLimits(config);
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
    normal_bandwidth_ = s["normal_bandwidth_hz"].as<double>(2.0);
    normal_max_rate_ = s["normal_max_rate_rad_s"].as<double>(1.5707963267948966);
    normal_max_deviation_ = s["normal_max_deviation_rad"].as<double>(0.5235987755982988);

    for (double value : {normal_bandwidth_, normal_max_rate_, normal_max_deviation_})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error("magnetic normal tracking parameters must be positive and finite");
    if (normal_max_deviation_ > 1.5707963267948966)
        throw std::runtime_error("magnetic normal_max_deviation_rad must not exceed pi/2");

    const double target = scalar("target_force_n");

    if (!(contact_off_ < contact_recover_ && contact_recover_ <= contact_cancel_ &&
            contact_cancel_ < contact_on_) ||
        contact_on_ >= maximum_force_ || target > maximum_force_ || ramp_time_ <= 0 ||
        release_delay_ <= 0 || maximum_speed_ <= 0 || maximum_distance_ <= 0)
        throw std::runtime_error("invalid magnetic thresholds, force, time or motion bounds");

    for (int i = 0; i < 3; ++i) {
        if (kp_motor_[i] < 0 || kp_motor_[i] > 500 || kd_motor_[i] < 0 ||
            torque_limit_[i] <= 0 || workspace_min_[i] >= workspace_max_[i])
            throw std::runtime_error("invalid magnetic joint/workspace bounds");
        if (torque_ref_as_position && kp_motor_[i] == 0)
            throw std::runtime_error(
                "magnetic_touch.kp_motor must be positive when torque_reference_mode=torque_ref_as_position");
    }
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

void MagneticTouchController::Start(double time, const std::array<double, 3>& toe_position) {
    const auto event = telemetry_.event_sequence + 1;
    telemetry_ = {};
    telemetry_.event_sequence = event;
    telemetry_.event_time = time;
    telemetry_.toe_reference = toe_position;
    anchor_position_ = previous_toe_position_ = toe_position;

    observer_.Reset();

    previous_time_ = phase_since_ = ramp_since_ = time;
    ramp_from_ = ramp_to_ = ramped_force_ = 0;
    armed_ = true;
}

void MagneticTouchController::Compute(double time,
    const std::array<double, 3>& joint_position,
    const std::array<double, 3>& joint_velocity,
    const std::array<double, 3>& motor_torque,
    const std::array<double, 3>& gravity_torque,
    const std::array<double, 3>& bias_torque,
    const std::array<double, 3>& passive_torque,
    const std::array<double, 3>& friction_limit,
    const std::array<double, 9>& mass_matrix,
    const std::array<double, 9>& coriolis_matrix,
    const std::array<double, 9>& jacobian,
    const std::array<double, 3>& toe_position,
    const std::array<double, 3>& joint_min,
    const std::array<double, 3>& joint_max,
    std::uint64_t measurement_sequence,
    double measurement_dt,
    double target_force,
    std::array<double, 3>& position_reference,
    std::array<double, 3>& torque) {
    if (!std::isfinite(time) || time < previous_time_ || !std::isfinite(target_force) ||
        target_force < 0 || target_force > maximum_force_)
        throw std::runtime_error("invalid magnetic time or target force");

    const double dt = time - previous_time_;
    previous_time_ = time;

    observer_.Update(measurement_sequence,
        measurement_dt,
        joint_velocity,
        motor_torque,
        mass_matrix,
        coriolis_matrix,
        jacobian,
        bias_torque,
        passive_torque,
        friction_limit,
        joint_position);

    telemetry_.observer_valid = observer_.valid();
    telemetry_.estimated_force = observer_.force();
    telemetry_.external_torque = observer_.torque();
    telemetry_.minimum_singular_value = observer_.minimumSingularValue();
    telemetry_.target_force = target_force;
    telemetry_.force_command = 0;
    telemetry_.torque_saturated = false;

    position_reference = joint_position;
    const auto& compensation = gravity_only_ ? gravity_torque : bias_torque;

    for (int i = 0; i < 3; ++i) {
        torque[i] = compensation[i];

        if (!std::isfinite(torque[i]))
            throw std::runtime_error("invalid magnetic joint feedback");
    }

    const auto bounds = [&]() {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(toe_position[i]) || toe_position[i] < workspace_min_[i] ||
                toe_position[i] > workspace_max_[i])
                return MagneticReason::Workspace;

            if (joint_max[i] > joint_min[i] &&
                (joint_position[i] < joint_min[i] + joint_margin_ ||
                    joint_position[i] > joint_max[i] - joint_margin_))
                return MagneticReason::JointLimit;
        }

        return MagneticReason::None;
    };

    MagneticReason invalid =
        !measurement_sequence ? MagneticReason::NoMeasurement
                              : (!observer_.valid() ? MagneticReason::ObserverInvalid : bounds());

    if (invalid != MagneticReason::None) {
        // A fresh GM needs two samples. Starting its momentum history is not a
        // contact fault; active contact still releases on any observer reset.
        const bool starting = invalid == MagneticReason::ObserverInvalid && observer_.warming_up() &&
            telemetry_.phase == MagneticPhase::Free && armed_;
        Release(invalid, time);
        if (starting) armed_ = true;
        telemetry_.toe_reference = toe_position;
    } else {
        const double magnitude = Norm(telemetry_.estimated_force);

        if (!armed_ && magnitude < contact_cancel_) {
            armed_ = true;

            if (telemetry_.reason == MagneticReason::NoMeasurement ||
                telemetry_.reason == MagneticReason::ObserverInvalid)
                Transition(MagneticPhase::Free, MagneticReason::None, time);
        }

        if (telemetry_.phase == MagneticPhase::Free) {
            telemetry_.toe_reference = toe_position;
            telemetry_.estimated_normal_force = magnitude;

            if (armed_ && magnitude > contact_on_)
                Transition(MagneticPhase::Candidate, MagneticReason::None, time);
        } else if (telemetry_.phase == MagneticPhase::Candidate) {
            telemetry_.toe_reference = toe_position;
            telemetry_.estimated_normal_force = magnitude;

            if (magnitude < contact_cancel_)
                Transition(MagneticPhase::Free, MagneticReason::None, time);
            else if (time - phase_since_ >= debounce_) {
                for (int i = 0; i < 3; ++i)
                    telemetry_.normal[i] = -telemetry_.estimated_force[i] / magnitude;

                anchor_position_ = previous_toe_position_ = toe_position;
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
            // Gate on the previous normal's projected force, not just |F|: a
            // tangential force must not rotate the normal to hide contact loss.
            if (telemetry_.phase == MagneticPhase::Following &&
                telemetry_.estimated_normal_force > contact_recover_ &&
                magnitude > contact_recover_ && dt > 0) {
                std::array<double, 3> measured_normal{};
                for (int i = 0; i < 3; ++i)
                    measured_normal[i] = -telemetry_.estimated_force[i] / magnitude;
                const double alignment = std::clamp(Dot(telemetry_.normal, measured_normal), -1.0, 1.0);
                if (alignment > std::cos(normal_max_deviation_)) {
                    const double angle = std::acos(alignment);
                    if (angle > 1e-8) {
                        const double alpha = -std::expm1(-2 * std::acos(-1.0) * normal_bandwidth_ * dt);
                        const double step = std::min(alpha * angle, normal_max_rate_ * dt);
                        // Spherical interpolation enforces the angular rate cap exactly.
                        const double from = std::sin(angle - step) / std::sin(angle);
                        const double to = std::sin(step) / std::sin(angle);
                        std::array<double, 3> updated{};
                        for (int i = 0; i < 3; ++i)
                            updated[i] = from * telemetry_.normal[i] + to * measured_normal[i];
                        const double length = Norm(updated);
                        if (length > 1e-8)
                            for (int i = 0; i < 3; ++i)
                                telemetry_.normal[i] = updated[i] / length;
                    }
                }
                telemetry_.estimated_normal_force = -Dot(telemetry_.normal, telemetry_.estimated_force);
            }
            std::array<double, 3> delta{}, velocity{}, toe_step{};

            for (int i = 0; i < 3; ++i) {
                toe_step[i] = toe_position[i] - previous_toe_position_[i];

                for (int j = 0; j < 3; ++j)
                    velocity[i] += jacobian[3 * i + j] * joint_velocity[j];
            }
            const double normal_step = Dot(telemetry_.normal, toe_step);
            for (int i = 0; i < 3; ++i) {
                anchor_position_[i] += toe_step[i] - telemetry_.normal[i] * normal_step;
                delta[i] = toe_position[i] - anchor_position_[i];
            }

            if (std::abs(Dot(telemetry_.normal, delta)) > maximum_distance_)
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
                            anchor_position_[i] + telemetry_.normal[i] * telemetry_.displacement;

                    std::array<double, 3> tracking{};

                    for (int i = 0; i < 3; ++i)
                        tracking[i] = telemetry_.toe_reference[i] - toe_position[i];

                    const double raw_force = ramped_force_ + force_gain_ * error +
                                             stiffness_ * Dot(telemetry_.normal, tracking) -
                                             cartesian_damping_ * Dot(telemetry_.normal, velocity);
                    telemetry_.force_command = std::clamp(raw_force, 0.0, maximum_force_);
                    bool saturated = raw_force != telemetry_.force_command;

                    for (int joint = 0; joint < 3; ++joint) {
                        double contact_torque = 0;

                        for (int axis = 0; axis < 3; ++axis)
                            contact_torque += jacobian[3 * axis + joint] * telemetry_.normal[axis] *
                                              telemetry_.force_command;

                        torque[joint] += contact_torque;
                        saturated |= std::abs(torque[joint]) > torque_limit_[joint];
                    }

                    if (saturated) {
                        telemetry_.displacement = old_displacement;  // Anti-windup.

                        for (int i = 0; i < 3; ++i)
                            telemetry_.toe_reference[i] =
                                anchor_position_[i] + telemetry_.normal[i] * old_displacement;
                    }
                }
            }
        }
    }

    if (telemetry_.phase == MagneticPhase::Free)
        telemetry_.toe_reference = toe_position;
    previous_toe_position_ = toe_position;

    for (int i = 0; i < 3; ++i) {
        telemetry_.torque_saturated |= std::abs(torque[i]) > torque_limit_[i];
        torque[i] = std::clamp(torque[i], -torque_limit_[i], torque_limit_[i]);
    }
}
}  // namespace qhrr05
