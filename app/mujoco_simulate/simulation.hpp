#pragma once

#include <array>
#include <string>
#include "shm_utils.hpp"

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
class VirtualMotors {
public:
    VirtualMotors(const std::string& hal_config_path,
        const std::array<double, 3>& encoder_origin,
        ShmData& shared);
    ~VirtualMotors();
    VirtualMotors(const VirtualMotors&) = delete;
    VirtualMotors& operator=(const VirtualMotors&) = delete;
    void Update(const mjModel* model, mjData* data);
    MotorData ReadFeedback() const;

private:
    void Reply(int axis, const openrobot::Payload& bytes);
    int socket_ = -1;
    std::array<unsigned, 3> ids_{};
    std::array<double, 3> encoder_origin_{};
    std::array<bool, 3> enabled_{};
    std::array<openrobot::PositionTarget, 3> targets_{};
    openrobot::MitProfile profile_{};
    ShmData& shared_;
};

std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model);
void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data);
ControlData UpdateControlIO(ShmData& shm,
    const mjModel* model,
    mjData* data,
    mjData* gravity_data,
    std::array<double, 3>& reference,
    std::uint64_t measurement_sequence = 0,
    const MotorData* can_feedback = nullptr);
std::array<double, 6> ReadContactGroundTruth(const mjModel* model, const mjData* data);
}  // namespace qhrr05
