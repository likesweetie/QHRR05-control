#include "interfaces/web_gui/bridge.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
std::string JsonString(const std::string& value) {
    std::ostringstream out;
    out << '"';

    for (unsigned char c : value) {
        if (c == '"' || c == '\\')
            out << '\\' << c;
        else if (c < 32)
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
        else
            out << c;
    }

    out << '"';
    return out.str();
}

std::string Number(double value) {
    if (!std::isfinite(value))
        return "null";

    std::ostringstream out;
    out << std::setprecision(15) << value;
    return out.str();
}
}  // namespace

WebGuiBridge::WebGuiBridge(const std::string& shm_name, const std::string& control_config)
    : mapping_(shm_name), simulation_(shm_name) {
    const auto config = YAML::LoadFile(control_config);
    maximum_force_ = config["magnetic_touch"]["maximum_force_n"].as<double>();

    if (!std::isfinite(maximum_force_) || maximum_force_ <= 0)
        throw std::runtime_error("web GUI maximum force must be finite and positive");
}

std::string WebGuiBridge::Snapshot() {
    MotorData motors;
    ModelData model;
    ControlData control;
    simulation_.Poll();
    const auto& simulation = simulation_.Latest();
    bool can_enabled, stopping, failed;
    {
        auto& shm = *mapping_.get();
        shm_utils::Lock lock(shm);
        motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        model = shm.model[shm.model_sequence % kShmBufferCount];
        control = shm.control[shm.control_sequence % kShmBufferCount];
        can_enabled = shm.can_enabled;
        stopping = shm.stopping;
        failed = shm.failed;
    }

    const auto now = shm_utils::MonotonicNs();
    const bool ready = !stopping && !failed && control.published_ns &&
                       (model.published_ns || control.mode == 6 || control.zero_set.required);
    std::ostringstream out;
    out << std::boolalpha << "{\"type\":\"state\",\"ready\":" << ready
        << ",\"can_enabled\":" << can_enabled << ",\"stopping\":" << stopping
        << ",\"failed\":" << failed
        << ",\"time\":" << (simulation.published_ns ? Number(simulation.time) : "null")
        << ",\"simulation_connected\":" << simulation_.Connected() << ",\"mode\":" << control.mode
        << ",\"requested_mode\":" << control.requested_mode
        << ",\"motion_sequence\":" << control.motion_sequence
        << ",\"active_motion_sequence\":" << control.active_motion_sequence
        << ",\"holding\":" << control.holding
        << ",\"zero_set\":{\"phase\":" << JsonString(ZeroSetPhaseName(control.zero_set.phase))
        << ",\"failure\":" << JsonString(ZeroSetFailureName(control.zero_set.failure))
        << ",\"axis\":" << JsonString(ZeroSetAxisName(control.zero_set.axis))
        << ",\"required\":" << control.zero_set.required << ",\"complete\":"
        << std::all_of(control.zero_set.calibrated.begin(),
               control.zero_set.calibrated.end(),
               [](bool v) {
                   return v;
               })
        << ",\"travel_rad\":" << Number(control.zero_set.travel_rad)
        << ",\"elapsed_sec\":" << Number(control.zero_set.elapsed_sec) << "}"
        << ",\"tracking_plane\":" << int(control.tracking_plane)
        << ",\"requested_tracking_plane\":" << int(control.requested_tracking_plane)
        << ",\"tracking_checking\":" << control.tracking_checking
        << ",\"tracking_check_progress\":" << Number(control.tracking_check_progress)
        << ",\"tracking_ik_failed\":" << control.tracking_ik_failed
        << ",\"tracking_torque_saturated\":" << control.tracking_torque_saturated
        << ",\"magnetic_phase\":" << JsonString(MagneticPhaseName(control.magnetic.phase))
        << ",\"target_force\":" << Number(control.target_force)
        << ",\"maximum_force\":" << Number(maximum_force_)
        << ",\"estimated_force\":" << Number(control.magnetic.estimated_normal_force)
        << ",\"contact\":"
        << (simulation.published_ns ? (simulation.contact ? "true" : "false") : "null")
        << ",\"joystick_connected\":" << control.joystick_connected << ",\"toe_position\":[";

    for (int i = 0; i < 3; ++i)
        out << (i ? "," : "") << (model.published_ns ? Number(model.toe_position[i]) : "null");

    out << "],\"motors\":[";

    for (int i = 0; i < 3; ++i) {
        const auto& feedback = motors.feedback[i];
        const bool measured =
            can_enabled ? motors.received_sequence[i] != 0 : motors.feedback_sequence != 0;
        const auto value = [&](double number) {
            return measured ? Number(number) : "null";
        };
        out << (i ? "," : "")
            << "{\"position\":" << value(motors.position_offset[i] + feedback.position)
            << ",\"velocity\":" << value(feedback.velocity)
            << ",\"current\":" << (can_enabled ? value(feedback.current) : "null")
            << ",\"torque\":" << value(motors.motor_torque[i]) << ",\"temperature\":"
            << (can_enabled && measured ? std::to_string(feedback.temperature) : "null")
            << ",\"enabled\":" << motors.enabled[i]
            << ",\"profile_verified\":" << motors.profile_verified[i]
            << ",\"received_sequence\":" << motors.received_sequence[i] << ",\"age_ms\":"
            << (motors.received_ns[i] && now >= motors.received_ns[i]
                       ? Number((now - motors.received_ns[i]) / 1e6)
                       : "null")
            << '}';
    }

    out << "]}";
    return out.str();
}

void WebGuiBridge::Run() {
    auto next_sample = shm_utils::MonotonicNs();

    for (;;) {
        const auto now = shm_utils::MonotonicNs();

        if (now >= next_sample) {
            std::cout << Snapshot() << std::endl;
            next_sample = now + 50000000;  // 20 Hz observer; not the control scheduler.
        }

        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const auto current = shm_utils::MonotonicNs();
        const int timeout = current >= next_sample ? 0 : int((next_sample - current) / 1000000);
        const int result = ::poll(&descriptor, 1, timeout);

        if (result < 0 && errno == EINTR)
            continue;

        if (result < 0 || descriptor.revents & (POLLERR | POLLNVAL))
            throw std::runtime_error("web GUI command pipe failed");

        if (descriptor.revents & (POLLIN | POLLHUP)) {
            char bytes[4096];
            const auto count = ::read(STDIN_FILENO, bytes, sizeof(bytes));

            if (count == 0)
                return;

            if (count < 0 && errno == EINTR)
                continue;

            if (count < 0)
                throw std::runtime_error("web GUI command read failed");

            throw std::runtime_error("telemetry bridge is read-only; send commands through Input");
        }

        if (!std::cout)
            throw std::runtime_error("web GUI state pipe closed");
    }
}
}  // namespace qhrr05
