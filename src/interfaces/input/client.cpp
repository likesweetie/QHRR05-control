#include "interfaces/input/client.hpp"
#include "ipc/shm_utils.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
InputClient::InputClient(const std::string& name)
    : path_("/tmp/qhrr05-input-" + std::to_string(getuid()) + "/" + name.substr(1) + ".sock"),
      owner_((std::uint64_t(getpid()) << 32) ^ shm_utils::MonotonicNs()) {
    if (name.size() < 2 || name[0] != '/' || name.find('/', 1) != std::string::npos)
        throw std::runtime_error("Input client requires a valid SHM session name");
    if (path_.size() >= sizeof(sockaddr_un::sun_path))
        throw std::runtime_error("Input client socket path is too long");
    worker_ = std::thread([this] {
        Run();
    });
}

InputClient::~InputClient() {
    ReleasePerturb();
    stopping_ = true;
    changed_.notify_all();
    worker_.join();
}

std::uint64_t InputClient::Submit(const std::string& operation) {
    std::lock_guard lock(mutex_);
    if (queue_.size() >= 64)
        throw std::runtime_error("Input command queue is full; command was not submitted");
    const auto id = ++next_id_;
    queue_.push_back({id, operation, false});
    changed_.notify_one();
    std::cout << "[input-client] requested id=" << id << ' ' << operation << std::endl;
    return id;
}

std::string InputClient::PerturbLine(const PerturbTarget& p) const {
    std::ostringstream line;
    line << std::setprecision(17) << "perturb " << p.owner << ' ' << p.active << ' '
         << p.body.data();
    for (const auto* values : {&p.reference_position, &p.reference_selection, &p.local_position})
        for (double value : *values)
            line << ' ' << value;
    for (double value : p.reference_quaternion)
        line << ' ' << value;
    return line.str();
}

void InputClient::SetPerturb(PerturbTarget target) {
    std::lock_guard lock(mutex_);
    if (!perturb_.active)
        perturb_reported_ = false;
    target.owner = owner_;
    perturb_ = target;
    perturb_updated_ns_ = shm_utils::MonotonicNs();
    changed_.notify_one();
}

void InputClient::ReleasePerturb() {
    std::lock_guard lock(mutex_);
    if (!perturb_.active)
        return;
    perturb_.active = 0;
    queue_.push_front({++next_id_, PerturbLine(perturb_), false});
    changed_.notify_one();
}

std::vector<InputDecision> InputClient::Decisions() {
    std::lock_guard lock(mutex_);
    std::vector<InputDecision> result;
    result.swap(decisions_);
    return result;
}

InputDecision InputClient::Exchange(const Request& request) {
    InputDecision result;
    result.id = request.id;
    result.operation = request.operation;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0)
        throw std::runtime_error("Input client socket creation failed");

    struct Close {
        int fd;

        ~Close() {
            ::close(fd);
        }
    } close{fd};

    const auto deadline = shm_utils::MonotonicNs() + 1000000000ULL;
    const auto ready = [&](short events) {
        while (shm_utils::MonotonicNs() < deadline) {
            pollfd descriptor{fd, events, 0};
            const int n = ::poll(&descriptor, 1, 20);
            if (n < 0 && errno != EINTR)
                shm_utils::Check(errno, "Input client poll");
            if (n > 0)
                return;
        }
        throw std::runtime_error(
            "Input decision unavailable after 1 s; approval is unknown (not retried)");
    };
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::copy(path_.begin(), path_.end(), address.sun_path);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        if (errno != EINPROGRESS)
            shm_utils::Check(errno, "Input connect");
        ready(POLLOUT);
        int error = 0;
        socklen_t size = sizeof(error);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) < 0)
            shm_utils::Check(errno, "Input connection status");
        shm_utils::Check(error, "Input connect completion");
    }
    const auto line = std::to_string(request.id) + ' ' + request.operation + '\n';
    std::size_t sent = 0;
    while (sent < line.size()) {
        ready(POLLOUT);
        const auto count = ::send(fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
        if (count < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (count <= 0)
            throw std::runtime_error("Input send failed; decision is unknown");
        sent += count;
    }
    std::string response;
    while (response.find('\n') == std::string::npos) {
        ready(POLLIN);
        char buffer[1024];
        const auto count = ::recv(fd, buffer, sizeof(buffer), 0);
        if (count < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (count <= 0)
            throw std::runtime_error("Input closed without a decision; approval is unknown");
        response.append(buffer, count);
        if (response.size() > 4096)
            throw std::runtime_error("Input decision is too large");
    }
    const auto json = YAML::Load(response);
    if (json["id"].as<std::uint64_t>() != request.id)
        throw std::runtime_error("Input decision id mismatch");
    const auto type = json["type"].as<std::string>();
    if (type == "ack" && json["stage"].as<std::string>() == "safety_approved") {
        result.approved = true;
        result.sequence = json["motion_sequence"].as<std::uint64_t>();
        result.message = "Safety approved";
    } else if (type == "error") {
        result.message = json["message"].as<std::string>();
    } else {
        throw std::runtime_error("unexpected Input decision format");
    }
    return result;
}

void InputClient::Run() {
    auto next_heartbeat = std::chrono::steady_clock::now();
    for (;;) {
        Request request;
        {
            std::unique_lock lock(mutex_);
            changed_.wait_for(lock, std::chrono::milliseconds(10), [&] {
                return stopping_ || !queue_.empty();
            });
            if (!queue_.empty()) {
                request = std::move(queue_.front());
                queue_.pop_front();
            } else if (stopping_) {
                return;
            } else if (perturb_.active &&
                       shm_utils::MonotonicNs() - perturb_updated_ns_ < 250000000ULL &&
                       std::chrono::steady_clock::now() >= next_heartbeat) {
                request = {++next_id_, PerturbLine(perturb_), perturb_reported_};
                perturb_reported_ = true;
                next_heartbeat = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
            } else {
                continue;
            }
        }
        InputDecision decision;
        try {
            decision = Exchange(request);
        } catch (const std::exception& error) {
            decision.id = request.id;
            decision.operation = request.operation;
            decision.unknown = true;
            decision.message = error.what();
        }
        if (!request.heartbeat || !decision.approved) {
            std::cout << "[input-client] id=" << decision.id << ' ' << decision.message
                      << std::endl;
            std::lock_guard lock(mutex_);
            if (decisions_.size() == 128)
                decisions_.erase(decisions_.begin());
            decisions_.push_back(std::move(decision));
        }
    }
}
}  // namespace qhrr05
