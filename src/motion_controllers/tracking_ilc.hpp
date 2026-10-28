#pragma once

#include "ipc/control_types.hpp"
#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace qhrr05 {
// Periodic torque ILC owned by TrackingController; no simulator/IPC dependency.
class TrackingILC {
public:
    explicit TrackingILC(const std::string& config_path);
    void Reset();
    void Hold();
    void Compute(double elapsed);
    void Record(const std::array<double, 3>& joint_position_error,
        const std::array<double, 3>& joint_velocity_error,
        double toe_error_m, bool controller_saturated);

    bool enabled() const {
        return enabled_;
    }

    TrackingILCState state() const {
        return state_;
    }

    const std::array<double, 3>& torque() const {
        return torque_;
    }

    std::uint64_t updates() const {
        return updates_;
    }

    std::uint64_t skipped_cycles() const {
        return skipped_cycles_;
    }

    bool limited() const {
        return limited_;
    }

    double last_rmse_m() const {
        return last_rmse_m_;
    }
private:
    void FinishCycle(bool consecutive);
    bool enabled_ = false, collecting_ = false, saturated_ = false, limited_ = false;
    TrackingILCState state_ = TrackingILCState::Off;
    double period_ = 0, learning_start_ = 0, blend_sec_ = 0;
    double retention_ = 1, last_elapsed_ = -1;
    double error_squared_ = 0, last_rmse_m_ = std::numeric_limits<double>::quiet_NaN();
    int bins_ = 0, lead_bins_ = 0, smoothing_passes_ = 0, bin_ = 0;
    std::int64_t cycle_ = -1;
    std::uint64_t updates_ = 0, skipped_cycles_ = 0, samples_ = 0;
    std::array<double, 3> position_gain_{}, velocity_gain_{}, torque_limit_{}, step_limit_{}, torque_{};
    std::vector<std::array<double, 3>> table_, previous_, learning_sum_, candidate_, scratch_;
    std::vector<unsigned> counts_;
};
}  // namespace qhrr05
