#include "interfaces/web_gui/bridge.hpp"
#include "interfaces/observations.hpp"

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

template <class T, std::size_t N> std::string ArrayJson(const std::array<T, N>& values) {
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < N; ++i)
        out << (i ? "," : "") << Number(values[i]);
    return out.str() + ']';
}
}  // namespace

WebGuiBridge::WebGuiBridge(const std::string& shm_name, const std::string& control_config)
    : mapping_(shm_name) {
    {
        auto& shm = *mapping_.get();
        // Immutable session metadata; do not acquire/repair the control mutex here.
        origin_ = shm.observation_origin;
        if (shm.simulation_available)
            simulation_ = std::make_unique<simulation_ipc::Client>(shm_name);
    }
    const auto config = YAML::LoadFile(control_config);
    maximum_force_ = config["magnetic_touch"]["maximum_force_n"].as<double>();
    if (const auto zero = config["zero_set"]) {
        std::ostringstream data;
        data << '{';
        bool first = true;
        for (const char* key :
            {"direction", "stop_position_rad", "speed_rad_s", "contact_torque_nm"}) {
            if (!first)
                data << ',';
            first = false;
            data << JsonString(key) << ':';
            const auto values = zero[key].as<std::vector<double>>();
            if (values.size() != 3)
                throw std::runtime_error("invalid zero-set axis configuration");
            data << '[';
            for (std::size_t i = 0; i < values.size(); ++i)
                data << (i ? "," : "") << Number(values[i]);
            data << ']';
        }
        zero_config_json_ = data.str() + '}';
    }

    if (!std::isfinite(maximum_force_) || maximum_force_ <= 0)
        throw std::runtime_error("web GUI maximum force must be finite and positive");
}

std::string WebGuiBridge::Snapshot() {
    if (!mapping_.IsCurrent())
        throw std::runtime_error(
            "SHM session ended or was replaced; restart the web GUI for the current daemon");
    MotorData motors;
    ModelData model;
    ControlData control;
    if (simulation_)
        simulation_->Poll();
    const auto simulation = simulation_ ? simulation_->Latest() : SimulationData{};
    bool can_enabled, stopping, failed;
    std::uint64_t motor_sequence, model_sequence, control_sequence;
    {
        auto& shm = *mapping_.get();
        shm_utils::Lock lock(shm);
        motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        model = shm.model[shm.model_sequence % kShmBufferCount];
        control = shm.control[shm.control_sequence % kShmBufferCount];
        can_enabled = shm.can_enabled;
        stopping = shm.stopping;
        failed = shm.failed;
        motor_sequence = shm.motor_sequence;
        model_sequence = shm.model_sequence;
        control_sequence = shm.control_sequence;
    }

    const auto now = shm_utils::MonotonicNs();
    const bool ready = !stopping && !failed && control.published_ns;
    std::ostringstream out;
    out << std::boolalpha << "{\"type\":\"state\",\"ready\":" << ready
        << ",\"can_enabled\":" << can_enabled << ",\"stopping\":" << stopping
        << ",\"failed\":" << failed << ",\"time\":"
        << (simulation_ ? (simulation.published_ns ? Number(simulation.time) : "null")
                        : Number(control.control_time))
        << ",\"simulation_connected\":" << (simulation_ && simulation_->Connected())
        << ",\"observed_ns\":" << now << ",\"shm\":{\"abi\":" << kShmAbiVersion
        << ",\"bytes\":" << kShmByteSize << ",\"motor_sequence\":" << motor_sequence
        << ",\"model_sequence\":" << model_sequence << ",\"control_sequence\":" << control_sequence
        << '}' << ",\"controller_age_ms\":"
        << (control.published_ns ? Number((now - control.published_ns) / 1e6) : "null")
        << ",\"transport_running\":" << motors.running
        << ",\"command_sequence\":" << motors.command_sequence
        << ",\"consumed_command_sequence\":" << motors.consumed_command_sequence
        << ",\"command_age_ms\":"
        << (motors.command_published_ns ? Number((now - motors.command_published_ns) / 1e6)
                                        : "null")
        << ",\"mode\":" << control.mode << ",\"simulation_available\":" << bool(simulation_)
        << ",\"observation_origin\":" << JsonString(ObservationOriginName(origin_))
        << ",\"drive_state\":" << JsonString(DriveStateName(control.drive_state))
        << ",\"drive_request_sequence\":" << control.active_drive_request_sequence
        << ",\"drive_fault\":" << JsonString(control.drive_fault_reason.data())
        << ",\"transport_fault\":" << JsonString(TransportFaultName(motors.transport_fault))
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
        << ",\"maximum_force\":" << Number(maximum_force_) << ",\"estimated_force\":"
        << (control.mode == 2 && control.magnetic.observer_valid
                   ? Number(control.magnetic.estimated_normal_force)
                   : "null")
        << ",\"contact\":"
        << (simulation.published_ns ? (simulation.contact ? "true" : "false") : "null")
        << ",\"joystick_connected\":" << control.joystick_connected << ",\"toe_position\":[";

    for (int i = 0; i < 3; ++i)
        out << (i ? "," : "") << (model.valid ? Number(model.toe_position[i]) : "null");

    out << "],\"motors\":[";

    for (int i = 0; i < 3; ++i) {
        const auto& feedback = motors.feedback[i];
        const auto value = [&](double number, std::uint32_t signal) {
            return HasSignals(motors, i, signal) ? Number(number) : "null";
        };
        const auto status = [&](std::uint32_t signal, double value) {
            if (!(motors.supported[i] & signal))
                return "unsupported";
            if (!motors.received_sequence[i])
                return "waiting";
            if (!std::isfinite(value))
                return "invalid";
            if (!(motors.valid[i] & signal))
                return "stale";
            return "valid";
        };
        out << (i ? "," : "") << "{\"position\":"
            << (motors.coordinate_valid[i]
                       ? value(motors.position_offset[i] + feedback.position, EncoderPosition)
                       : "null")
            << ",\"velocity\":" << value(feedback.velocity, JointVelocity)
            << ",\"current\":" << value(feedback.current, MotorCurrent)
            << ",\"torque\":" << value(motors.motor_torque[i], ConvertedTorque)
            << ",\"temperature\":" << value(feedback.temperature, MotorTemperature)
            << ",\"encoder_position\":" << value(feedback.position, EncoderPosition)
            << ",\"coordinate_valid\":" << motors.coordinate_valid[i] << ",\"position_status\":"
            << JsonString(
                   motors.coordinate_valid[i]
                       ? status(EncoderPosition, feedback.position + motors.position_offset[i])
                       : "uncalibrated")
            << ",\"velocity_status\":" << JsonString(status(JointVelocity, feedback.velocity))
            << ",\"current_status\":" << JsonString(status(MotorCurrent, feedback.current))
            << ",\"torque_status\":" << JsonString(status(ConvertedTorque, motors.motor_torque[i]))
            << ",\"temperature_status\":"
            << JsonString(status(MotorTemperature, feedback.temperature))
            << ",\"enabled\":" << motors.enabled[i]
            << ",\"command_enabled\":" << motors.command_enabled[i]
            << ",\"command\":{\"position\":" << Number(motors.command[i].position)
            << ",\"velocity\":" << Number(motors.command[i].velocity)
            << ",\"kp\":" << Number(motors.command[i].kp)
            << ",\"kd\":" << Number(motors.command[i].kd)
            << ",\"tau_ff\":" << Number(motors.command[i].tau_ff) << '}'
            << ",\"reference\":" << Number(motors.reference[i])
            << ",\"supported\":" << motors.supported[i] << ",\"valid\":" << motors.valid[i]
            << ",\"position_offset\":" << Number(motors.position_offset[i])
            << ",\"measurement_ns\":" << motors.measurement_ns[i]
            << ",\"tx_frames\":" << motors.tx_frames[i] << ",\"rx_frames\":" << motors.rx_frames[i]
            << ",\"invalid_frames\":" << motors.invalid_frames[i]
            << ",\"profile_verified\":" << motors.profile_verified[i]
            << ",\"received_sequence\":" << motors.received_sequence[i] << ",\"age_ms\":"
            << (motors.received_ns[i] && now >= motors.received_ns[i]
                       ? Number((now - motors.received_ns[i]) / 1e6)
                       : "null")
            << '}';
    }

    out << "],\"model\":{\"valid\":" << model.valid
        << ",\"acceleration_valid\":" << model.acceleration_valid
        << ",\"input_measurement_sequence\":" << model.input_measurement_sequence
        << ",\"acceleration\":" << ArrayJson(model.joint_acceleration)
        << ",\"mass\":" << ArrayJson(model.mass) << ",\"jacobian\":" << ArrayJson(model.jacobian)
        << ",\"gravity\":" << ArrayJson(model.gravity_torque)
        << ",\"bias\":" << ArrayJson(model.bias_torque)
        << ",\"passive\":" << ArrayJson(model.passive_torque)
        << ",\"joint_min\":" << ArrayJson(model.joint_min)
        << ",\"joint_max\":" << ArrayJson(model.joint_max)
        << "},\"zero_set_config\": " << zero_config_json_ << '}';
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
