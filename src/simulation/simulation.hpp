#pragma once

#include <array>
#include <string>
#include "ipc/shm_utils.hpp"
#include "motor_can_protocol/openrobot_protocol.hpp"
#include "simulation/virtual_iq_sensor.hpp"

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
class VirtualMotors {
public:
    VirtualMotors(const std::string& hal_config_path,
        const std::string& simulation_config_path,
        const std::array<double, 3>& encoder_origin,
        ShmData& shared);
    ~VirtualMotors();
    VirtualMotors(const VirtualMotors&) = delete;
    VirtualMotors& operator=(const VirtualMotors&) = delete;
    void Update(const mjModel* model, mjData* data);
    MotorData ReadFeedback() const;

    double torque_limit_nm(int axis) const {
        return torque_max_[axis];
    }

    bool require_zero_position_setup() const {
        return require_zero_;
    }

private:
    enum class MotorMode { Released, Enabled, MitControl, RmdTorque };

    void Reply(int axis, const openrobot::Payload& bytes);
    int socket_ = -1;
    bool require_zero_ = false;
    std::array<unsigned, 3> ids_{};
    std::array<MotorMode, 3> modes_{};
    std::array<MitCommand, 3> targets_{};
    // MIT runtime zero is separate from the physical encoder's origin.
    std::array<double, 3> mit_origin_{}, zero_hold_remaining_{};
    double motor_time_ = -1;
    std::array<VirtualIqSensor, 3> iq_sensors_;
    std::array<double, 3> actual_kt_output_{}, gear_ratio_{}, pole_pairs_{}, electrical_zero_{};
    std::array<double, 3> reported_iq_{};
    double sensor_time_ = -1;
    std::array<double, 3> velocity_max_{}, torque_max_{}, kt_output_{};
    std::array<unsigned, 3> kd_scale_{};
    std::array<openrobot::Payload, 3> rom_parameters_{};  // Per-motor C4 responses.
    ShmData& shared_;
};

std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model);
void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data);
// publish_model is only for isolated numerical test fixtures; live control owns ModelData.
ControlData UpdateControlIO(ShmData& shm,
    const mjModel* model,
    mjData* data,
    mjData* gravity_data,
    std::array<double, 3>& reference,
    std::uint64_t measurement_sequence = 0,
    const MotorData* can_feedback = nullptr,
    bool publish_model = false);
std::array<double, 6> ReadContactGroundTruth(const mjModel* model, const mjData* data);
}  // namespace qhrr05
