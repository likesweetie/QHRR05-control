#include "shm_logger/log_schema.hpp"

namespace qhrr05::logging {
std::optional<DataTamer2::RegistrationID> RegisterState(
    DataTamer2::LogChannel& c, LogFrame& f, bool simulation) {
    c.registerValue("snapshot/capture_ns", &f.capture_ns);
    c.registerValue("snapshot/lock_acquired_ns", &f.lock_acquired_ns);
    c.registerValue("snapshot/lock_released_ns", &f.lock_released_ns);
    c.registerValue("shm/abi_version", &f.abi_version);
    c.registerValue("shm/motor_sequence", &f.motor_sequence);
    c.registerValue("shm/model_sequence", &f.model_sequence);
    c.registerValue("shm/control_sequence", &f.control_sequence);
    if (simulation)
        c.registerValue("simulation/ipc_sequence", &f.simulation_sequence);
    c.registerValue("shm/stopping", &f.stopping);
    c.registerValue("shm/failed", &f.failed);
    c.registerValue("shm/can_enabled", &f.can_enabled);
    if (simulation)
        c.registerValue("simulation/finished", &f.simulation_finished);
    c.registerValue("simulation/valid", &f.simulation_valid);
    c.registerValue("motor", &f.motor);
    c.registerValue("model", &f.model);
    c.registerValue("control", &f.control);

    // Human-readable axis aliases; raw structures retain every field for replay.
    const char* axes[] = {"HR", "HP", "KP"};
    for (int i = 0; i < 3; ++i) {
        const std::string prefix = std::string("motor/") + axes[i] + "/";
        c.registerValue(prefix + "position_rad", &f.motor.feedback[i].position);
        c.registerValue(prefix + "velocity_rad_s", &f.motor.feedback[i].velocity);
        c.registerValue(prefix + "current_a", &f.motor.feedback[i].current);
        c.registerValue(prefix + "torque_nm", &f.motor.motor_torque[i]);
        c.registerValue(prefix + "command", &f.motor.command[i]);
    }

    // Hardware logs omit GT entirely; validity also covers pre-publication startup.
    if (simulation) {
        const auto id = c.registerValue("simulation/ground_truth", &f.simulation);
        c.setEnabled(id, false);
        return id;
    }
    return std::nullopt;
}
}  // namespace qhrr05::logging
