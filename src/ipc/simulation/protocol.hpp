#pragma once

#include <array>
#include <cstdint>

// Simulation-only messages. This header is deliberately independent of the hardware SHM.
enum class SimulationAction { None, Pause, Reset, Step, Perturb };

struct PerturbTarget {
    std::uint64_t owner = 0;
    int active = 0;  // MuJoCo mjPERT_TRANSLATE=1 / mjPERT_ROTATE=2; zero releases.
    std::array<char, 32> body{};
    std::array<double, 3> reference_position{}, reference_selection{}, local_position{};
    std::array<double, 4> reference_quaternion{1, 0, 0, 0};
};

struct SimulationCommand {
    SimulationAction action = SimulationAction::None;
    int pause = 2;  // 0: run, 1: pause, 2: toggle.
    PerturbTarget perturb{};
};

inline constexpr std::uint64_t kPerturbLeaseNs = 500000000;

// TaskController-local approval state. Never mapped into shared memory.
struct SimulationApproval {
    std::uint64_t pause_sequence = 0, perturb_expires_ns = 0;
    bool paused = false;
    PerturbTarget perturb{};
};

// Physical truth and object/contact events are published only on simulation IPC.
struct SimulationData {
    std::uint64_t published_ns = 0, run_id = 0, sequence = 0, transport_drops = 0;
    bool finished = false;
    double time = 0;
    std::array<double, 3> position_gt{}, torque_gt{}, toe_position{};
    std::array<double, 3> contact_force{}, contact_torque{}, object_position{};
    std::array<double, 3> velocity_gt{};
    std::array<double, 4> object_quaternion{1, 0, 0, 0};
    bool paused = false;
    std::uint64_t applied_input_sequence = 0, applied_pause_sequence = 0, applied_step_sequence = 0,
                  perturb_owner = 0;
    int perturb_active = 0;
    std::array<double, 6> perturb_wrench{};
    bool contact = false;
    std::uint64_t contact_event_sequence = 0;
    double contact_event_time = 0;
    // Startup policy only; no physical joint angle is needed by calibration.
    bool require_zero_position_setup = false;
};
