#include "interfaces/input/interface.hpp"

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

InputInterface::InputInterface(const std::string& shm_name) {
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

        if (parsed && operation == "mode") {
            int plane = -1;
            parsed = bool(input >> command.mode >> plane);
            command.plane = static_cast<TrackingPlane>(plane);
            command.set_mode = true;
        } else if (parsed && operation == "force")
            parsed = bool(input >> command.force);
        else if (parsed && operation == "pause") {
            command.simulation.action = SimulationAction::Pause;
            parsed = bool(input >> command.simulation.pause);
        } else if (parsed && operation == "reset")
            command.simulation.action = SimulationAction::Reset;
        else if (parsed && operation == "step")
            command.simulation.action = SimulationAction::Step;
        else if (parsed && operation == "perturb") {
            command.simulation.action = SimulationAction::Perturb;
            auto& target = command.simulation.perturb;
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

        return command;
    }

    return std::nullopt;
}

void InputInterface::Reply(
    const InputCommand& command, const std::string& rejection, std::uint64_t motion_sequence) {
    std::ostringstream out;
    out << "{\"id\":" << command.id;

    if (rejection.empty())
        out << ",\"type\":\"ack\",\"stage\":\"safety_approved\",\"motion_sequence\":"
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
