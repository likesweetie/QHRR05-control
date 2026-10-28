#include "motion_controllers/tracking_ilc.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TrackingILC::TrackingILC(const std::string& config_path) {
    const auto config = YAML::LoadFile(config_path);
    const auto tracking = config["tracking"];
    const auto ilc = tracking["ilc"];
    if (ilc["learning_rate"].IsDefined())
        throw std::runtime_error("ILC learning_rate was removed; multiply it into the position/velocity gains");

    if (ilc["torque_limit_nm"].IsDefined() || ilc["step_limit_nm"].IsDefined())
        throw std::runtime_error("ILC limits were renamed to correction_limit_nm/update_limit_nm");

    enabled_ = ilc["enabled"].as<bool>();

    if (!enabled_)
        return;

    period_ = tracking["trajectory"]["period_sec"].as<double>();
    const double ramp = tracking["trajectory"]["ramp_sec"].as<double>();
    const double frequency = config["control"]["frequency_hz"].as<double>();
    const int warmup_cycles = ilc["warmup_cycles"].as<int>();
    bins_ = ilc["phase_bins"].as<int>();
    lead_bins_ = ilc["phase_lead_bins"].as<int>();
    smoothing_passes_ = ilc["smoothing_passes"].as<int>();
    retention_ = ilc["retention"].as<double>();
    blend_sec_ = ilc["blend_sec"].as<double>();

    for (double value : {period_, ramp, frequency, blend_sec_})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error("ILC period/ramp/frequency/blend_sec must be positive and finite");

    if (bins_ < 16 || bins_ > 8192 || bins_ > period_ * frequency ||
        warmup_cycles < 0 || warmup_cycles > 100 || lead_bins_ < 0 || lead_bins_ > bins_ / 4 ||
        smoothing_passes_ < 0 || smoothing_passes_ > 32 ||
        !std::isfinite(retention_) || retention_ <= 0 || retention_ > 1 || blend_sec_ > period_ / 4)
        throw std::runtime_error("invalid ILC phase grid, warmup, filter, learning or blend setting");

    const auto axes = [&](const char* key) {
        const auto values = ilc[key].as<std::vector<double>>();

        if (values.size() != 3)
            throw std::runtime_error(std::string("ILC needs three joint values: ") + key);

        std::array<double, 3> result;
        std::copy(values.begin(), values.end(), result.begin());

        for (double value : result)
            if (!std::isfinite(value) || value < 0)
                throw std::runtime_error(std::string("invalid ILC joint value: ") + key);

        return result;
    };
    position_gain_ = axes("position_gain_nm_rad");
    velocity_gain_ = axes("velocity_gain_nm_s_rad");
    torque_limit_ = axes("correction_limit_nm");
    step_limit_ = axes("update_limit_nm");

    for (int i = 0; i < 3; ++i)
        if (position_gain_[i] <= 0 || torque_limit_[i] <= 0 || step_limit_[i] <= 0 ||
            step_limit_[i] > torque_limit_[i])
            throw std::runtime_error("ILC position gain/torque/step limits must be positive, step <= torque limit");

    // Steady profile-clock phase is (elapsed - ramp/2)/period. Start at its first
    // center crossing after the speed ramp and any requested warmup laps.
    learning_start_ = .5 * ramp + (std::ceil(.5 * ramp / period_) + warmup_cycles) * period_;

    if (!std::isfinite(learning_start_))
        throw std::runtime_error("ILC phase-aligned learning start is not representable");

    table_.resize(bins_);
    previous_.resize(bins_);
    learning_sum_.resize(bins_);
    candidate_.resize(bins_);
    scratch_.resize(bins_);
    counts_.resize(bins_);
    Reset();
}

void TrackingILC::Reset() {
    for (auto* values : {&table_, &previous_, &learning_sum_, &candidate_, &scratch_})
        std::fill(values->begin(), values->end(), std::array<double, 3>{});

    std::fill(counts_.begin(), counts_.end(), 0);
    torque_.fill(0);
    cycle_ = -1;
    last_elapsed_ = -1;
    updates_ = skipped_cycles_ = samples_ = 0;
    error_squared_ = 0;
    last_rmse_m_ = std::numeric_limits<double>::quiet_NaN();
    collecting_ = saturated_ = limited_ = false;
    state_ = enabled_ ? TrackingILCState::Warmup : TrackingILCState::Off;
}

void TrackingILC::Hold() {
    if (!enabled_)
        return;

    collecting_ = false;
    torque_.fill(0);
    state_ = TrackingILCState::IKHold;
}

void TrackingILC::FinishCycle(bool consecutive) {
    previous_ = table_;
    const bool complete = consecutive &&
        std::all_of(counts_.begin(), counts_.end(), [](unsigned count) { return count != 0; });

    if (complete)
        last_rmse_m_ = std::sqrt(error_squared_ / samples_);

    if (!complete || saturated_) {
        ++skipped_cycles_;
        state_ = complete ? TrackingILCState::SkippedSaturation : TrackingILCState::SkippedSamples;
        std::cerr << "[tracking ILC] cycle " << cycle_ << " skipped: "
                  << TrackingILCStateName(state_) << '\n';
        return;
    }

    for (int i = 0; i < bins_; ++i) {
        const int source = (i + lead_bins_) % bins_;

        for (int joint = 0; joint < 3; ++joint)
            candidate_[i][joint] = retention_ * table_[i][joint] +
                learning_sum_[source][joint] / counts_[source];
    }

    // Circular symmetric Q filter: no phase lag and no special seam endpoint.
    for (int pass = 0; pass < smoothing_passes_; ++pass) {
        for (int i = 0; i < bins_; ++i)
            for (int joint = 0; joint < 3; ++joint)
                scratch_[i][joint] = .25 * candidate_[(i + bins_ - 1) % bins_][joint] +
                    .5 * candidate_[i][joint] + .25 * candidate_[(i + 1) % bins_][joint];

        candidate_.swap(scratch_);
    }

    limited_ = false;

    for (int i = 0; i < bins_; ++i) {
        for (int joint = 0; joint < 3; ++joint) {
            const double delta = candidate_[i][joint] - table_[i][joint];
            const double updated = table_[i][joint] +
                std::clamp(delta, -step_limit_[joint], step_limit_[joint]);
            limited_ |= std::abs(delta) > step_limit_[joint] || std::abs(updated) > torque_limit_[joint];
            table_[i][joint] = std::clamp(updated, -torque_limit_[joint], torque_limit_[joint]);
        }
    }

    ++updates_;
    state_ = TrackingILCState::Active;
}

void TrackingILC::Compute(double elapsed) {
    collecting_ = false;
    torque_.fill(0);

    if (!enabled_ || state_ == TrackingILCState::IKHold)
        return;

    if (!std::isfinite(elapsed) || elapsed < 0 || elapsed < last_elapsed_)
        throw std::runtime_error("ILC requires monotonic elapsed time; restart Tracking after a rewind");

    last_elapsed_ = elapsed;

    if (elapsed < learning_start_)
        return;

    const double lap = (elapsed - learning_start_) / period_;

    if (!std::isfinite(lap) || lap >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("ILC cycle index is not representable");

    const auto cycle = static_cast<std::int64_t>(std::floor(lap));
    const double phase = lap - cycle;

    if (cycle != cycle_) {
        if (cycle_ >= 0) {
            const auto distance = cycle - cycle_;
            FinishCycle(distance == 1);
            skipped_cycles_ += distance - 1;
        } else {
            state_ = TrackingILCState::Collecting;
        }

        cycle_ = cycle;
        std::fill(learning_sum_.begin(), learning_sum_.end(), std::array<double, 3>{});
        std::fill(counts_.begin(), counts_.end(), 0);
        error_squared_ = 0;
        samples_ = 0;
        saturated_ = false;
    }

    bin_ = std::min(static_cast<int>(phase * bins_), bins_ - 1);
    collecting_ = true;
    const double coordinate = phase * bins_ - .5;  // Table nodes are bin centers.
    const int left = static_cast<int>(std::floor(coordinate));
    const int a = (left + bins_) % bins_, b = (a + 1) % bins_;
    const double fraction = coordinate - left;
    const double s = std::clamp(phase * period_ / blend_sec_, 0.0, 1.0);
    const double blend = s * s * s * (10 + s * (-15 + 6 * s));

    for (int joint = 0; joint < 3; ++joint) {
        const double before = previous_[a][joint] + fraction * (previous_[b][joint] - previous_[a][joint]);
        const double after = table_[a][joint] + fraction * (table_[b][joint] - table_[a][joint]);
        torque_[joint] = before + blend * (after - before);

        if (!std::isfinite(torque_[joint]))
            throw std::runtime_error("non-finite ILC correction");
    }
}

void TrackingILC::Record(const std::array<double, 3>& joint_position_error,
    const std::array<double, 3>& joint_velocity_error, double toe_error_m, bool controller_saturated) {
    if (!collecting_)
        return;

    if (!std::isfinite(toe_error_m) || toe_error_m < 0)
        throw std::runtime_error("invalid ILC toe error");

    for (int joint = 0; joint < 3; ++joint) {
        const double signal = position_gain_[joint] * joint_position_error[joint] +
            velocity_gain_[joint] * joint_velocity_error[joint];

        if (!std::isfinite(signal))
            throw std::runtime_error("non-finite ILC learning signal");

        learning_sum_[bin_][joint] += signal;

        if (!std::isfinite(learning_sum_[bin_][joint]))
            throw std::runtime_error("non-finite accumulated ILC learning signal");
    }

    ++counts_[bin_];
    ++samples_;
    error_squared_ += toe_error_m * toe_error_m;

    if (!std::isfinite(error_squared_))
        throw std::runtime_error("non-finite accumulated ILC toe error");

    saturated_ |= controller_saturated;
}
}  // namespace qhrr05
