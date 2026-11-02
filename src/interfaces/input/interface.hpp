#pragma once

#include "ipc/shm_utils.hpp"
#include <array>
#include <optional>
#include <memory>
#include <string>

namespace qhrr05 {
struct InputCommand {
    std::uint64_t id = 0;
    bool set_mode = false;
    int mode = 5;
    TrackingPlane plane = TrackingPlane::XY;
    double force = 0;
    DriveAction drive_action = DriveAction::None;
    int motor_axis = -1;
    bool auxiliary = false;
    bool anticogging_status = false, set_anticogging_phase = false, set_anticogging_amplitude = false;
    int anticogging_order = 21;
    double phase_delta = 0, amplitude_delta = 0;
    bool rmd_status = false, set_rmd_damping = false;
    double rmd_damping_constant = 0;
    bool tracking_ilc_status = false, set_tracking_ilc = false;
    int ilc_phase_lead_bins = 0, ilc_smoothing_passes = 0;
    double ilc_retention = 1;
};

struct InputContext {
    double progress_time = 0;
    bool start_calibration = false;
    bool calibration_blocked = false;
};

// All producers publish requests. Only TaskController's safety gate approves them.
class InputInterface {
public:
    explicit InputInterface(const std::string& shm_name);
    ~InputInterface();
    InputInterface(const InputInterface&) = delete;
    InputInterface& operator=(const InputInterface&) = delete;
    InputContext Poll();
    std::uint64_t ApproveAuxiliary();
    std::optional<InputCommand> Receive();
    void Reply(
        const InputCommand& command, const std::string& rejection, std::uint64_t motion_sequence,
        const std::string& data_json = {});

    static std::uint64_t RequestMode(ShmData& shm, int mode, TrackingPlane plane, double force);
    static void RequestDrive(ShmData& shm, DriveAction action);
    static void RequestForce(ShmData& shm, double force);
    static void PublishJoystick(ShmData& shm, const ControlData& input);

private:
    struct Environment;
    std::unique_ptr<Environment> environment_;
    double progress_time_ = 0;
    std::uint64_t previous_poll_ns_ = 0;
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
