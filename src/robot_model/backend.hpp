#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace qhrr05 {
struct KinematicState {
    std::array<double, 3> position{};
    std::array<double, 9> jacobian{};
};

struct RobotModelState {
    std::array<double, 3> joint_position{}, joint_velocity{}, toe_position{};
    std::array<double, 3> gravity{}, bias{}, passive{}, friction{}, damping{}, joint_min{},
        joint_max{};
    std::array<double, 9> mass{}, jacobian{};
};

// Thread-confined mathematical backend. Inputs are measured encoder states,
// never simulator ground truth. Evaluating a target must not overwrite state().
class ModelBackend {
public:
    virtual ~ModelBackend() = default;
    virtual const RobotModelState& Update(
        const std::array<double, 3>& q, const std::array<double, 3>& dq) = 0;
    virtual const RobotModelState& state() const = 0;
    virtual KinematicState Evaluate(const std::array<double, 3>& q, bool jacobian = true) = 0;
    virtual std::array<double, 3> ConvectiveAcceleration(
        const std::array<double, 3>& q, const std::array<double, 3>& dq) = 0;
    virtual std::array<double, 3> InverseDynamics(const std::array<double, 3>& q,
        const std::array<double, 3>& dq,
        const std::array<double, 3>& ddq) = 0;
    virtual std::uint64_t updates() const = 0;
};

// Resolves model_path relative to the control YAML. Failure is explicit;
// no simulator-state or alternative-model fallback exists.
std::unique_ptr<ModelBackend> MakeMujocoBackend(const std::string& control_config);
}  // namespace qhrr05
