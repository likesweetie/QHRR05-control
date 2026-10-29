#pragma once

#include "ipc/simulation/protocol.hpp"

#include "ipc/shm_utils.hpp"
#include <array>
#include <optional>
#include <string>

namespace qhrr05 {
struct InputCommand {
    std::uint64_t id = 0;
    bool set_mode = false;
    int mode = 5;
    TrackingPlane plane = TrackingPlane::XY;
    double force = 0;
    SimulationCommand simulation{};
};

// All producers publish requests. Only TaskController's safety gate approves them.
class InputInterface {
public:
    explicit InputInterface(const std::string& shm_name);
    ~InputInterface();
    InputInterface(const InputInterface&) = delete;
    InputInterface& operator=(const InputInterface&) = delete;
    std::optional<InputCommand> Receive();
    void Reply(
        const InputCommand& command, const std::string& rejection, std::uint64_t motion_sequence);

    static std::uint64_t RequestMode(ShmData& shm, int mode, TrackingPlane plane, double force);
    static void RequestForce(ShmData& shm, double force);
    static void PublishJoystick(ShmData& shm, const ControlData& input);

private:
    int listener_ = -1, replying_ = -1;
    std::string path_;

    struct Client {
        int fd = -1;
        std::string bytes;
        std::uint64_t opened_ns = 0;
    };

    std::array<Client, 8> clients_{};
};
}  // namespace qhrr05
