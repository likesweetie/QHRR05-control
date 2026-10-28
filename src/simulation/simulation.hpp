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

    double torque_limit_nm() const {
        return torque_max_;
    }

private:
    void Reply(int axis, const openrobot::Payload& bytes);
    int socket_ = -1;
    std::array<unsigned, 3> ids_{};
    std::array<double, 3> encoder_origin_{};
    std::array<bool, 3> enabled_{};
    std::array<MitCommand, 3> targets_{};
    std::array<VirtualIqSensor, 3> iq_sensors_;
    std::array<double, 3> actual_kt_output_{}, gear_ratio_{}, pole_pairs_{}, electrical_zero_{};
    std::array<double, 3> reported_iq_{};
    double sensor_time_ = -1;
    double velocity_max_ = 0, torque_max_ = 0, kt_output_ = 0;
    unsigned kd_scale_ = 0;
    openrobot::Payload rom_parameters_{};  // Simulated motor's C4 response.
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
