#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace qhrr05 {
// Host-side damping from cumulative MIT position differences and a 30 Hz velocity LPF.
class DampingController {
public:
    explicit DampingController(const std::string& config_path);
    void Reset();
    void Compute(const std::array<double, 3>& position,
        const std::array<std::uint64_t, 3>& sequence,
        const std::array<std::uint64_t, 3>& sample_ns,
        std::array<double, 3>& position_reference,
        std::array<double, 3>& torque);
    const std::array<double, 3>& velocity() const { return velocity_; }  // Filtered rad/s.
private:
    std::array<double, 3> gain_{}, limit_{}, previous_position_{}, velocity_{};
    std::array<std::uint64_t, 3> sequence_{}, sample_ns_{};
    std::array<bool, 3> initialized_{};
};
}  // namespace qhrr05
