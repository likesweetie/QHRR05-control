#include "interfaces/input/interface.hpp"
#include "interfaces/input/simulation.hpp"
#include "ipc/simulation/channel.hpp"
#include <cmath>
#include <cstring>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>

namespace qhrr05 {
namespace {
void CheckActive(const ShmData& shm) {
    if (shm.stopping || shm.failed)
        throw std::runtime_error("Input session is stopped or failed");
}

std::string JsonString(const std::string& value) {
    std::string result = "\"";

    for (unsigned char c : value) {
        if (c == '"' || c == '\\')
            result += '\\';

        result += c < 32 ? ' ' : char(c);
    }

    return result + '"';
}
}  // namespace

void ValidateSimulationInput(const SimulationCommand& command,
    const SimulationData& state,
    const SimulationApproval& previous,
    std::uint64_t now) {
    if (!state.published_ns)
        throw std::runtime_error("safety rejected simulation request: simulation is not ready");
    if (command.action == SimulationAction::Pause) {
        if (command.pause < 0 || command.pause > 2)
            throw std::runtime_error("safety rejected simulation request: invalid pause value");
        return;
    }
    if (command.action == SimulationAction::Reset)
        return;
    if (command.action == SimulationAction::Step) {
        const bool paused =
            previous.pause_sequence > state.applied_pause_sequence ? previous.paused : state.paused;
        if (!paused)
            throw std::runtime_error("single step requires a paused simulation");
        return;
    }
    if (command.action != SimulationAction::Perturb)
        throw std::runtime_error("safety rejected simulation request: invalid operation");
    const auto& p = command.perturb;
    if (!p.owner || p.active < 0 || p.active > 2)
        throw std::runtime_error("safety rejected perturbation owner or action");
    const char* bodies[] = {"base", "FL_hip", "FL_thigh", "FL_calf", "touch_object"};
    const bool named = p.body.back() == 0 &&
                       std::any_of(std::begin(bodies), std::end(bodies), [&](const char* name) {
                           return std::strcmp(name, p.body.data()) == 0;
                       });
    if (!named)
        throw std::runtime_error("safety rejected perturbation: unknown QHRR body");
    for (const auto* values : {&p.reference_position, &p.reference_selection, &p.local_position})
        for (double value : *values)
            if (!std::isfinite(value))
                throw std::runtime_error("safety rejected perturbation: non-finite position");
    double norm = 0;
    for (double value : p.reference_quaternion) {
        if (!std::isfinite(value))
            throw std::runtime_error("safety rejected perturbation: non-finite quaternion");
        norm += value * value;
    }
    if (std::abs(norm - 1) > .001)
        throw std::runtime_error("safety rejected perturbation: quaternion must be unit length");
    if (previous.perturb.active && previous.perturb_expires_ns > now &&
        previous.perturb.owner != p.owner)
        throw std::runtime_error("perturbation is owned by another viewer; release it first");
}

struct InputInterface::Environment {
    explicit Environment(const std::string& name) : channel(name, true) {
    }

    simulation_ipc::Client channel;
    SimulationApproval approval{};
    SimulationCommand pending{};
    bool initialized = false;
};

InputContext InputInterface::Poll() {
    InputContext result;
    const auto now = shm_utils::MonotonicNs();
    bool paused = false;
    if (environment_) {
        auto& env = *environment_;
        env.channel.Poll();
        const auto& state = env.channel.Latest();
        if (state.published_ns) {
            result.start_calibration = !env.initialized && state.require_zero_position_setup;
            env.initialized = true;
            paused = env.channel.Connected() && !state.finished && state.paused &&
                     now >= state.published_ns && now - state.published_ns < 500000000ULL;
            result.calibration_blocked = state.contact;
        }
    }
    if (previous_poll_ns_ && !paused)
        progress_time_ += (now - previous_poll_ns_) / 1e9;
    previous_poll_ns_ = now;
    result.progress_time = progress_time_;
    return result;
}

std::uint64_t InputInterface::ApproveAuxiliary() {
    if (!environment_)
        throw std::runtime_error("simulation commands are unavailable for this session");
    auto& env = *environment_;
    auto request = env.pending;
    const auto& state = env.channel.Latest();
    ValidateSimulationInput(request, state, env.approval, shm_utils::MonotonicNs());
    if (request.action == SimulationAction::Pause) {
        const bool paused = env.approval.pause_sequence > state.applied_pause_sequence
                                ? env.approval.paused
                                : state.paused;
        request.pause = request.pause == 2 ? !paused : request.pause;
    }
    const auto sequence = env.channel.SendApproved(request);
    if (request.action == SimulationAction::Pause) {
        env.approval.paused = request.pause;
        env.approval.pause_sequence = sequence;
        env.approval.perturb.active = 0;
    } else if (request.action == SimulationAction::Reset) {
        env.approval.perturb.active = 0;
    } else if (request.action == SimulationAction::Perturb) {
        env.approval.perturb = request.perturb;
        env.approval.perturb_expires_ns =
            request.perturb.active ? shm_utils::MonotonicNs() + kPerturbLeaseNs : 0;
    }
    return sequence;
}

InputInterface::InputInterface(const std::string& shm_name) {
    {
        shm_utils::Mapping mapping(shm_name);
        shm_utils::Lock lock(*mapping.get());
        if (mapping.get()->simulation_available)
            environment_ = std::make_unique<Environment>(shm_name);
    }
    const std::string directory = "/tmp/qhrr05-input-" + std::to_string(::getuid());

    if (::mkdir(directory.c_str(), 0700) < 0 && errno != EEXIST)
        shm_utils::Check(errno, "Input directory");

    struct stat info {};

    if (::lstat(directory.c_str(), &info) < 0 || !S_ISDIR(info.st_mode) ||
        info.st_uid != ::getuid() || (info.st_mode & 077) != 0)
        throw std::runtime_error(
            "Input socket directory must be owned by this user with mode 0700");

    path_ = directory + "/" + shm_name.substr(1) + ".sock";
    sockaddr_un address{};
    address.sun_family = AF_UNIX;

    if (path_.size() >= sizeof(address.sun_path))
        throw std::runtime_error("Input socket path is too long");

    std::copy(path_.begin(), path_.end(), address.sun_path);
    listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);

    if (listener_ < 0)
        shm_utils::Check(errno, "Input socket");

    if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        const int error = errno;
        ::close(listener_);
        listener_ = -1;
        shm_utils::Check(error, "Input bind (existing sockets are not reused or removed)");
    }

    if (::listen(listener_, 8) < 0) {
        const int error = errno;
        ::close(listener_);
        listener_ = -1;
        ::unlink(path_.c_str());
        shm_utils::Check(error, "Input listen");
    }
}

InputInterface::~InputInterface() {
    if (replying_ >= 0)
        ::close(replying_);

    for (auto& client : clients_)
        if (client.fd >= 0)
            ::close(client.fd);

    if (listener_ >= 0) {
        ::close(listener_);
        ::unlink(path_.c_str());
    }
}

std::optional<InputCommand> InputInterface::Receive() {
    if (replying_ >= 0)
        throw std::runtime_error("Input request was not answered");

    for (auto& client : clients_) {
        if (client.fd < 0) {
            client.fd = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);

            if (client.fd < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                continue;

            if (client.fd < 0 && errno == EINTR)
                continue;

            if (client.fd < 0)
                shm_utils::Check(errno, "Input accept");

            ucred credentials{};
            socklen_t length = sizeof(credentials);

            if (::getsockopt(client.fd, SOL_SOCKET, SO_PEERCRED, &credentials, &length) < 0 ||
                credentials.uid != ::getuid()) {
                ::close(client.fd);
                client = {};
                continue;
            }

            client.opened_ns = shm_utils::MonotonicNs();
        }

        char buffer[1024];
        const auto count = ::recv(client.fd, buffer, sizeof(buffer), MSG_DONTWAIT);

        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            if (shm_utils::MonotonicNs() - client.opened_ns > 1000000000ULL) {
                ::close(client.fd);
                client = {};
            }

            continue;
        }

        if (count <= 0) {
            ::close(client.fd);
            client = {};
            continue;
        }

        client.bytes.append(buffer, count);
        const auto end = client.bytes.find('\n');

        if (end == std::string::npos && client.bytes.size() <= 1024)
            continue;

        replying_ = client.fd;
        const auto line = client.bytes;
        client = {};
        InputCommand command;
        std::istringstream input(line);
        std::string operation;
        bool parsed = end != std::string::npos && line.size() <= 1024 &&
                      bool(input >> command.id >> operation) && command.id > 0 &&
                      command.id <= 2147483647;

        SimulationCommand simulation;
        if (parsed && operation == "mode") {
            int plane = -1;
            parsed = bool(input >> command.mode >> plane);
            command.plane = static_cast<TrackingPlane>(plane);
            command.set_mode = true;
        } else if (parsed &&
                   (operation == "arm" || operation == "release" || operation == "clear-fault")) {
            command.drive_action = operation == "arm"       ? DriveAction::Arm
                                   : operation == "release" ? DriveAction::Release
                                                            : DriveAction::ClearFault;
        } else if (parsed && (operation == "motor-enable" || operation == "motor-disable")) {
            parsed = bool(input >> command.motor_axis);
            command.drive_action =
                operation == "motor-enable" ? DriveAction::EnableMotor : DriveAction::DisableMotor;
        } else if (parsed && operation == "force")
            parsed = bool(input >> command.force);
        else if (parsed && operation == "pause") {
            simulation.action = SimulationAction::Pause;
            parsed = bool(input >> simulation.pause);
        } else if (parsed && operation == "reset")
            simulation.action = SimulationAction::Reset;
        else if (parsed && operation == "step")
            simulation.action = SimulationAction::Step;
        else if (parsed && operation == "perturb") {
            simulation.action = SimulationAction::Perturb;
            auto& target = simulation.perturb;
            std::string body;
            parsed = bool(input >> target.owner >> target.active >> body);
            if (body.size() >= target.body.size())
                parsed = false;
            else
                std::copy(body.begin(), body.end(), target.body.begin());
            for (auto* values :
                {&target.reference_position, &target.reference_selection, &target.local_position})
                for (double& value : *values)
                    parsed &= bool(input >> value);
            for (double& value : target.reference_quaternion)
                parsed &= bool(input >> value);
        } else
            parsed = false;

        input >> std::ws;

        if (!parsed || !input.eof()) {
            Reply(command, "invalid Input command format", 0);
            continue;
        }

        if (simulation.action != SimulationAction::None) {
            if (!environment_) {
                Reply(command, "simulation commands are unavailable for this session", 0);
                continue;
            }
            environment_->pending = simulation;
            command.auxiliary = true;
        }
        return command;
    }

    return std::nullopt;
}

void InputInterface::Reply(
    const InputCommand& command, const std::string& rejection, std::uint64_t motion_sequence) {
    if (!command.id)
        return;
    std::ostringstream out;
    out << "{\"id\":" << command.id;

    if (rejection.empty())
        out << ",\"type\":\"ack\",\"stage\":\"safety_approved\",\"motion_sequence\":"
            << motion_sequence
            << (command.drive_action != DriveAction::None ? ",\"drive_sequence\":"
                                                          : ",\"request_sequence\":")
            << motion_sequence;
    else
        out << ",\"type\":\"error\",\"message\":" << JsonString(rejection);

    out << "}\n";
    const auto bytes = out.str();
    const auto sent = ::send(replying_, bytes.data(), bytes.size(), MSG_DONTWAIT | MSG_NOSIGNAL);

    if (sent != static_cast<ssize_t>(bytes.size()))
        std::cerr << "[input] safety decision reply failed; request is not retried\n";

    ::close(replying_);
    replying_ = -1;
}

std::uint64_t InputInterface::RequestMode(
    ShmData& shm, int mode, TrackingPlane plane, double force) {
    shm_utils::Lock lock(shm);
    CheckActive(shm);
    auto& input = shm_utils::Next(shm.control, shm.control_sequence);
    input.requested_mode = mode;
    input.requested_tracking_plane = plane;
    input.target_force = force;
    ++input.motion_sequence;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
    return input.motion_sequence;
}

void InputInterface::RequestDrive(ShmData& shm, DriveAction action) {
    shm_utils::Lock lock(shm);
    CheckActive(shm);
    auto& input = shm_utils::Next(shm.control, shm.control_sequence);
    input.requested_drive_action = action;
    ++input.drive_request_sequence;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
}

void InputInterface::RequestForce(ShmData& shm, double force) {
    shm_utils::Lock lock(shm);
    CheckActive(shm);
    auto& input = shm_utils::Next(shm.control, shm.control_sequence);
    input.target_force = force;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
}

void InputInterface::PublishJoystick(ShmData& shm, const ControlData& sample) {
    shm_utils::Lock lock(shm);

    if (shm.stopping)
        return;

    CheckActive(shm);
    auto& input = shm_utils::Next(shm.control, shm.control_sequence);
    input.joystick_axes = sample.joystick_axes;
    input.joystick_buttons = sample.joystick_buttons;
    input.joystick_l2 = sample.joystick_l2;
    input.joystick_r2 = sample.joystick_r2;
    input.joystick_connected = sample.joystick_connected;
    ++input.joystick_sequence;
    input.joystick_published_ns = shm_utils::MonotonicNs();
    ++shm.control_sequence;
    shm_utils::Notify(shm);
}
}  // namespace qhrr05
