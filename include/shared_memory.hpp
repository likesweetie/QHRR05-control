#pragma once

#include <cstddef>
#include <cstdint>

inline constexpr std::size_t kShmNameBytes = 32;
inline constexpr std::size_t kShmMaxQpos = 64;
inline constexpr std::size_t kShmMaxQvel = 64;
inline constexpr std::size_t kShmMaxCtrl = 64;
inline constexpr std::size_t kShmMaxSensorData = 512;

struct ShmData
{
    char name[kShmNameBytes] = {0};
    std::uint64_t counter{0};

    // Monotonic sequence numbers for best-effort sync across processes.
    std::uint64_t state_seq{0};
    std::uint64_t command_seq{0};
    std::uint64_t applied_command_seq{0};

    // Current model dimensions and sim time.
    double sim_time{0.0};
    std::int32_t nq{0};
    std::int32_t nv{0};
    std::int32_t nu{0};
    std::int32_t nsensordata{0};

    // State written by MuJoCo simulate process.
    double qpos[kShmMaxQpos] = {0.0};
    double qvel[kShmMaxQvel] = {0.0};
    // Reserved ABI fields: qhrr-05 does not publish IMU measurements here.
    double quat[4] = {1.0, 0.0, 0.0, 0.0};  // w, x, y, z
    double ang_vel[3] = {0.0, 0.0, 0.0};    // wx, wy, wz
    double sensordata[kShmMaxSensorData] = {0.0};
    double ctrl_applied[kShmMaxCtrl] = {0.0};  // MuJoCo actuator_force, not constraint force.

    // Joint-space target written by external controller process.
    double q_target[kShmMaxCtrl] = {0.0};

    // Command channels for high-level velocity targets (e.g. joystick).
    double lin_vel_target[3] = {0.0, 0.0, 0.0};
    double ang_vel_target[3] = {0.0, 0.0, 0.0};

    bool a_button=false;
    bool b_button=false;
    bool x_button=false;
    bool y_button=false;

    bool lb_button=false;
    bool rb_button=false;

    bool back_button=false;
    bool start_button=false;
    bool guide_button=false;

    bool l3_button=false;
    bool r3_button=false;

};
