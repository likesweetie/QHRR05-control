#include "motion_controllers/zero_set_controller.hpp"
#include "control/limits.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
std::array<double, 3> Axes(const YAML::Node& settings, const char* key) {
    const auto node = settings[key];
    if (!node || !node.IsSequence() || node.size() != 3)
        throw std::runtime_error(std::string("zero_set.") + key + " requires three axis values");
    std::array<double, 3> values;
    for (int i = 0; i < 3; ++i) {
        values[i] = node[i].as<double>();
        if (!std::isfinite(values[i]))
            throw std::runtime_error(std::string("zero_set.") + key + " must be finite");
    }
    return values;
}

double Positive(const YAML::Node& settings, const char* key) {
    const double value = settings[key].as<double>();
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(std::string("zero_set.") + key + " must be positive and finite");
    return value;
}
}  // namespace

ZeroSetController::ZeroSetController(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const auto s = config["zero_set"];
    if (!s)
        throw std::runtime_error("zero_set settings are required for stop calibration");
    direction_ = Axes(s, "direction");
    stop_position_ = Axes(s, "stop_position_rad");
    speed_ = Axes(s, "speed_rad_s");
    threshold_ = Axes(s, "contact_torque_nm");
    effort_limit_ = Axes(s, "maximum_push_nm");
    kp_ = Axes(s, "kp");
    kd_ = Axes(s, "kd");
    travel_limit_ = Axes(s, "maximum_travel_rad");
    timeout_ = Axes(s, "timeout_sec");
    debounce_ = Positive(s, "contact_duration_sec");
    velocity_threshold_ = Positive(s, "contact_velocity_rad_s");
    backoff_ = Positive(s, "backoff_rad");
    settle_tolerance_ = Positive(s, "settle_tolerance_rad");
    feedback_timeout_ = Positive(s, "feedback_timeout_sec");
    const auto limits = ReadControlTorqueLimits(config);
    for (int i = 0; i < 3; ++i) {
        if ((direction_[i] != -1 && direction_[i] != 1) || speed_[i] <= 0 || kp_[i] <= 0 ||
            kd_[i] < 0 || threshold_[i] <= 0 || effort_limit_[i] <= threshold_[i] ||
            effort_limit_[i] > limits[i] || travel_limit_[i] <= backoff_ || timeout_[i] <= 0)
            throw std::runtime_error(
                "invalid zero_set direction, gains, travel, timeout or torque bounds");
    }
    if (settle_tolerance_ >= backoff_ ||
        debounce_ >= *std::min_element(timeout_.begin(), timeout_.end()))
        throw std::runtime_error("zero_set settling/debounce must fit backoff/timeout");
}

void ZeroSetController::Start(double time, const std::array<double, 3>& encoder) {
    if (!std::isfinite(time))
        throw std::runtime_error("invalid zero-set start time");
    for (double q : encoder)
        if (!std::isfinite(q))
            throw std::runtime_error("invalid zero-set encoder position");
    telemetry_ = {};
    telemetry_.phase = ZeroSetPhase::Seeking;
    telemetry_.axis = 0;
    reference_ = encoder;
    sequence_.fill(0);
    last_sample_time_.fill(time);
    last_time_ = time;
    BeginAxis(time, encoder);
}

void ZeroSetController::BeginAxis(double time, const std::array<double, 3>& encoder) {
    axis_time_ = time;
    start_ = encoder;
    contact_since_ = -1;
    telemetry_.phase = ZeroSetPhase::Seeking;
    std::cout << "[zero-set] SEEK " << ZeroSetAxisName(telemetry_.axis)
              << " direction=" << direction_[telemetry_.axis]
              << " threshold=" << threshold_[telemetry_.axis] << " Nm" << std::endl;
}

void ZeroSetController::Fail(ZeroSetFailure reason) {
    telemetry_.phase = ZeroSetPhase::Fault;
    telemetry_.failure = reason;
    std::cerr << "[zero-set] FAULT " << ZeroSetAxisName(telemetry_.axis) << ": "
              << ZeroSetFailureName(reason) << "; motor output released" << std::endl;
}

void ZeroSetController::Cancel() {
    if (telemetry_.phase == ZeroSetPhase::Seeking || telemetry_.phase == ZeroSetPhase::Backoff) {
        telemetry_.phase = ZeroSetPhase::Cancelled;
        std::cout << "[zero-set] CANCELLED; completed axis offsets retained" << std::endl;
    }
}

void ZeroSetController::Update(double time,
    const std::array<double, 3>& encoder,
    const std::array<double, 3>& velocity,
    const std::array<double, 3>& torque,
    const std::array<std::uint64_t, 3>& sample_sequence) {
    if (telemetry_.phase != ZeroSetPhase::Seeking && telemetry_.phase != ZeroSetPhase::Backoff &&
        telemetry_.phase != ZeroSetPhase::Complete)
        return;
    if (!std::isfinite(time) || time < last_time_)
        throw std::runtime_error("non-monotonic zero-set clock");
    const double dt = time - last_time_;
    last_time_ = time;
    std::array<bool, 3> fresh;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(encoder[i]) || !std::isfinite(velocity[i]) ||
            !std::isfinite(torque[i])) {
            Fail(ZeroSetFailure::InvalidFeedback);
            return;
        }
        fresh[i] = sample_sequence[i] != sequence_[i];
        if (fresh[i]) {
            sequence_[i] = sample_sequence[i];
            last_sample_time_[i] = time;
        }
        if (time - last_sample_time_[i] > feedback_timeout_) {
            Fail(ZeroSetFailure::FeedbackStale);
            return;
        }
    }
    if (telemetry_.phase == ZeroSetPhase::Complete)
        return;
    const int axis = telemetry_.axis;
    telemetry_.travel_rad = std::abs(encoder[axis] - start_[axis]);
    telemetry_.elapsed_sec = time - axis_time_;
    if (telemetry_.elapsed_sec > timeout_[axis]) {
        Fail(ZeroSetFailure::Timeout);
        return;
    }
    if (telemetry_.travel_rad > travel_limit_[axis]) {
        Fail(ZeroSetFailure::TravelLimit);
        return;
    }
    if (telemetry_.phase == ZeroSetPhase::Seeking) {
        // Limit both effort and accumulated position error at a blocked joint.
        const double lead = effort_limit_[axis] / kp_[axis];
        reference_[axis] = std::clamp(reference_[axis] + direction_[axis] * speed_[axis] * dt,
            encoder[axis] - lead,
            encoder[axis] + lead);
        if (!fresh[axis])
            return;
        const bool contact = direction_[axis] * torque[axis] >= threshold_[axis] &&
                             std::abs(velocity[axis]) <= velocity_threshold_;
        if (!contact) {
            contact_since_ = -1;
            return;
        }
        if (contact_since_ < 0)
            contact_since_ = time;
        if (time - contact_since_ < debounce_)
            return;
        telemetry_.offset[axis] = stop_position_[axis] - encoder[axis];
        telemetry_.calibrated[axis] = true;
        reference_[axis] = encoder[axis];
        backoff_target_ = encoder[axis] - direction_[axis] * backoff_;
        telemetry_.phase = ZeroSetPhase::Backoff;
        std::cout << "[zero-set] CALIBRATED " << ZeroSetAxisName(axis)
                  << " offset=" << telemetry_.offset[axis] << " rad; backing off" << std::endl;
    } else {
        const double step = speed_[axis] * dt;
        reference_[axis] += std::clamp(backoff_target_ - reference_[axis], -step, step);
        if (fresh[axis] && std::abs(reference_[axis] - backoff_target_) < 1e-9 &&
            std::abs(encoder[axis] - backoff_target_) <= settle_tolerance_ &&
            std::abs(velocity[axis]) <= velocity_threshold_) {
            if (++telemetry_.axis == 3) {
                telemetry_.phase = ZeroSetPhase::Complete;
                std::cout << "[zero-set] COMPLETE hip -> thigh -> calf; holding calibrated pose"
                          << std::endl;
            } else {
                BeginAxis(time, encoder);
            }
        }
    }
}
}  // namespace qhrr05
