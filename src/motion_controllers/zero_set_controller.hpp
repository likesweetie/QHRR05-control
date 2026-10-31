#pragma once

#include "ipc/control_types.hpp"
#include <array>
#include <string>

namespace qhrr05 {
// Works exclusively in encoder coordinates. No simulator truth or dynamics model
// is used to detect a stop; the configured stop angle defines the joint frame.
class ZeroSetController {
public:
    explicit ZeroSetController(const std::string& config_path);
    void Start(double time, const std::array<double, 3>& encoder);
    void Update(double time,
        const std::array<double, 3>& encoder,
        const std::array<double, 3>& velocity,
        const std::array<double, 3>& torque,
        const std::array<std::uint64_t, 3>& sample_sequence);
    void Cancel();
    void Fail(ZeroSetFailure reason);

    const ZeroSetTelemetry& telemetry() const {
        return telemetry_;
    }

    const std::array<double, 3>& reference() const {
        return reference_;
    }

    const std::array<double, 3>& kp() const {
        return kp_;
    }

    const std::array<double, 3>& kd() const {
        return kd_;
    }

    const std::array<double, 3>& effort_limit() const {
        return effort_limit_;
    }

    double feedback_timeout() const {
        return feedback_timeout_;
    }

private:
    void BeginAxis(double time, const std::array<double, 3>& encoder);
    std::array<double, 3> direction_{}, stop_position_{}, speed_{}, threshold_{}, effort_limit_{};
    std::array<double, 3> kp_{}, kd_{}, travel_limit_{}, timeout_{};
    std::array<double, 3> reference_{}, start_{}, last_sample_time_{};
    std::array<std::uint64_t, 3> sequence_{};
    double debounce_ = 0, velocity_threshold_ = 0, backoff_ = 0, settle_tolerance_ = 0;
    double feedback_timeout_ = 0, last_time_ = 0, axis_time_ = 0, contact_since_ = -1;
    double backoff_target_ = 0;
    ZeroSetTelemetry telemetry_{};
};
}  // namespace qhrr05
