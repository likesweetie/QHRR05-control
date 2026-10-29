#include "ipc/simulation/channel.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace qhrr05::simulation_ipc {
namespace {
constexpr std::uint64_t magic = 0x5148525253494d31ULL;
constexpr std::uint32_t version = 1;

enum Type : std::uint32_t { HelloObserver = 1, HelloController, State, Command };

struct Header {
    std::uint64_t magic_value = magic;
    std::uint32_t protocol_version = version, type = 0, size = 0, reserved = 0;
};

std::uint64_t Now() {
    timespec now{};
    if (clock_gettime(CLOCK_MONOTONIC, &now))
        throw std::runtime_error("simulation IPC clock failed");
    return std::uint64_t(now.tv_sec) * 1000000000 + now.tv_nsec;
}

std::string Path(const std::string& session, bool create) {
    if (session.size() < 2 || session[0] != '/' || session.find('/', 1) != std::string::npos)
        throw std::runtime_error("simulation IPC requires a valid session name");
    const auto directory = "/tmp/qhrr05-simulation-" + std::to_string(getuid());
    if (create) {
        if (mkdir(directory.c_str(), 0700) && errno != EEXIST)
            throw std::runtime_error("simulation IPC directory: " + std::string(strerror(errno)));

        struct stat info {};

        if (lstat(directory.c_str(), &info) || !S_ISDIR(info.st_mode) || info.st_uid != getuid() ||
            (info.st_mode & 077))
            throw std::runtime_error("simulation IPC directory must be user-owned and mode 0700");
    }
    const auto path = directory + "/" + session.substr(1) + ".sock";
    if (path.size() >= sizeof(sockaddr_un::sun_path))
        throw std::runtime_error("simulation IPC path is too long");
    return path;
}

sockaddr_un Address(const std::string& path) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::copy(path.begin(), path.end(), address.sun_path);
    return address;
}

int LockEndpoint(const std::string& path) {
    const int fd = open((path + ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0)
        throw std::runtime_error("simulation IPC ownership lock: " + std::string(strerror(errno)));

    struct stat info {};

    if (fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_uid != getuid() ||
        (info.st_mode & 077)) {
        close(fd);
        throw std::runtime_error("simulation IPC ownership lock is not a private user-owned file");
    }
    if (flock(fd, LOCK_EX | LOCK_NB)) {
        close(fd);
        throw std::runtime_error("simulation IPC endpoint is owned by an active server: " + path);
    }
    return fd;
}

void RemoveStaleEndpoint(const std::string& path) {
    struct stat before {};

    if (lstat(path.c_str(), &before)) {
        if (errno == ENOENT)
            return;
        throw std::runtime_error(
            "cannot inspect simulation IPC endpoint: " + std::string(strerror(errno)));
    }
    if (!S_ISSOCK(before.st_mode) || before.st_uid != getuid())
        throw std::runtime_error(
            "refusing to remove a non-socket or foreign simulation IPC endpoint: " + path);
    const int probe = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (probe < 0)
        throw std::runtime_error("simulation IPC liveness probe failed");
    const auto address = Address(path);
    const int result = connect(probe, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    const int error = errno;
    close(probe);
    if (result == 0 || error != ECONNREFUSED)
        throw std::runtime_error(
            "simulation IPC endpoint is active or cannot be verified stale; preserving: " + path);

    struct stat current {};

    if (lstat(path.c_str(), &current) || current.st_ino != before.st_ino ||
        current.st_dev != before.st_dev)
        throw std::runtime_error(
            "simulation IPC endpoint changed during stale check; preserving: " + path);
    if (unlink(path.c_str()))
        throw std::runtime_error("cannot remove verified stale simulation IPC endpoint: " +
                                 std::string(strerror(errno)));
    std::cerr << "[simulation-ipc] removed stale endpoint (no listener): " << path << '\n';
}

bool Send(int fd, Type type, const void* payload, std::size_t size) {
    Header header;
    header.type = type;
    header.size = size;
    iovec parts[2]{{&header, sizeof(header)}, {const_cast<void*>(payload), size}};
    msghdr message{};
    message.msg_iov = parts;
    message.msg_iovlen = size ? 2 : 1;
    const auto count = sendmsg(fd, &message, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return false;
    if (count != static_cast<ssize_t>(sizeof(header) + size))
        throw std::runtime_error("simulation IPC send: " + std::string(strerror(errno)));
    return true;
}

// -1: no packet, 0: closed, otherwise message type. Invalid versions never fall back.
int Receive(int fd, void* payload, std::size_t capacity, std::size_t& size) {
    std::array<char, 4096> bytes;
    const auto count = recv(fd, bytes.data(), bytes.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return -1;
    if (!count)
        return 0;
    if (count < 0)
        throw std::runtime_error("simulation IPC receive: " + std::string(strerror(errno)));
    if (count < ssize_t(sizeof(Header)) || count > ssize_t(bytes.size()))
        throw std::runtime_error("simulation IPC invalid packet length");
    Header header;
    std::memcpy(&header, bytes.data(), sizeof(header));
    if (header.magic_value != magic || header.protocol_version != version ||
        header.size != count - sizeof(Header) || header.size > capacity)
        throw std::runtime_error("simulation IPC protocol/version mismatch");
    size = header.size;
    if (size)
        std::memcpy(payload, bytes.data() + sizeof(Header), size);
    return header.type;
}
}  // namespace

Server::Server(const std::string& session) : path_(Path(session, true)) {
    const int ownership = LockEndpoint(path_);
    try {
        RemoveStaleEndpoint(path_);
        listener_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        const auto address = Address(path_);
        if (listener_ < 0 ||
            bind(listener_, reinterpret_cast<const sockaddr*>(&address), sizeof(address))) {
            const auto reason = std::string(strerror(errno));
            if (listener_ >= 0)
                close(listener_);
            listener_ = -1;
            throw std::runtime_error("simulation IPC bind: " + reason);
        }
        if (listen(listener_, 16)) {
            close(listener_);
            listener_ = -1;
            unlink(path_.c_str());
            throw std::runtime_error("simulation IPC listen failed");
        }
        std::cout << "[simulation-ipc] endpoint=" << path_ << " protocol=" << version << std::endl;
        lock_fd_ = ownership;
    } catch (...) {
        close(ownership);
        throw;
    }
}

Server::~Server() {
    for (auto& peer : peers_)
        if (peer.fd >= 0)
            close(peer.fd);
    if (listener_ >= 0) {
        close(listener_);
        unlink(path_.c_str());
    }
    if (lock_fd_ >= 0)
        close(lock_fd_);
}

void Server::Flush(Peer& peer) {
    if (peer.pending && latest_.sequence && peer.role) {
        auto state = latest_;
        state.transport_drops = peer.dropped;
        if (Send(peer.fd, State, &state, sizeof(state)))
            peer.pending = false;
    }
}

std::vector<ApprovedCommand> Server::Poll() {
    std::vector<ApprovedCommand> commands;
    for (auto& peer : peers_) {
        if (peer.fd < 0) {
            const int fd = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                    continue;
                throw std::runtime_error("simulation IPC accept failed");
            }
            ucred credentials{};
            socklen_t size = sizeof(credentials);
            if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &credentials, &size) ||
                credentials.uid != getuid()) {
                close(fd);
                std::cerr << "[simulation-ipc] rejected peer credentials\n";
                continue;
            }
            peer = {fd, 0, false};
        }
        try {
            for (int packet = 0; packet < 16; ++packet) {
                ApprovedCommand command;
                std::size_t size = 0;
                const int type = Receive(peer.fd, &command, sizeof(command), size);
                if (type < 0)
                    break;
                if (!type) {
                    close(peer.fd);
                    peer = {};
                    break;
                }
                if (!peer.role && (type == HelloObserver || type == HelloController) && !size) {
                    peer.role = type;
                    peer.pending = latest_.sequence != 0;
                } else if (peer.role == HelloController && type == Command &&
                           size == sizeof(command)) {
                    commands.push_back(command);
                } else {
                    throw std::runtime_error("simulation IPC invalid message or peer role");
                }
            }
            if (peer.fd >= 0)
                Flush(peer);
        } catch (const std::exception& error) {
            std::cerr << "[simulation-ipc] peer ERROR: " << error.what() << '\n';
            close(peer.fd);
            peer = {};
        }
    }
    return commands;
}

void Server::Publish(SimulationData state) {
    state.sequence = latest_.sequence + 1;
    state.transport_drops = drops_;
    latest_ = state;
    for (auto& peer : peers_) {
        if (peer.fd < 0 || !peer.role)
            continue;
        if (peer.pending) {
            ++drops_;
            ++peer.dropped;
        }
        peer.pending = true;
        try {
            Flush(peer);
        } catch (const std::exception& error) {
            std::cerr << "[simulation-ipc] subscriber ERROR: " << error.what() << '\n';
            close(peer.fd);
            peer = {};
        }
    }
}

void Server::Finish() {
    auto state = latest_;
    state.finished = true;
    state.published_ns = Now();
    Publish(state);
}

Client::Client(const std::string& session, bool controller)
    : controller_(controller), path_(Path(session, false)) {
}

Client::~Client() {
    if (fd_ >= 0)
        close(fd_);
}

void Client::Close(const std::string& reason) {
    if (fd_ >= 0)
        close(fd_);
    fd_ = -1;
    hello_ = false;
    status_ = reason;
    retry_ns_ = Now() + 250000000;
}

void Client::Poll() {
    if (latest_.finished)
        return;
    if (fd_ < 0) {
        if (Now() < retry_ns_)
            return;
        fd_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0)
            throw std::runtime_error("simulation IPC client socket failed");
        const auto address = Address(path_);
        if (connect(fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) &&
            errno != EINPROGRESS) {
            Close("simulation IPC unavailable: " + std::string(strerror(errno)));
            return;
        }
    }
    try {
        if (!hello_) {
            if (!Send(fd_, controller_ ? HelloController : HelloObserver, nullptr, 0))
                return;
            hello_ = true;
            status_ = "connected";
        }
        for (int packet = 0; packet < 128; ++packet) {
            SimulationData state;
            std::size_t size = 0;
            const int type = Receive(fd_, &state, sizeof(state), size);
            if (type < 0)
                break;
            if (!type) {
                Close(latest_.finished ? "finished" : "simulation IPC disconnected");
                break;
            }
            if (type != State || size != sizeof(state))
                throw std::runtime_error("simulation IPC state format mismatch");
            latest_ = state;
            status_ = state.finished ? "finished" : "live";
        }
    } catch (const std::exception& error) {
        Close(error.what());
        throw;
    }
}

std::uint64_t Client::SendApproved(const SimulationCommand& command) {
    if (!controller_)
        throw std::runtime_error("simulation IPC observer cannot send approved commands");
    Poll();
    if (!Connected() || latest_.finished)
        throw std::runtime_error("simulation command not delivered: " + status_);
    ApprovedCommand packet;
    packet.sequence = ++next_sequence_;
    packet.issued_ns = Now();
    packet.expires_ns = command.action == SimulationAction::Perturb && command.perturb.active
                            ? packet.issued_ns + kPerturbLeaseNs
                            : 0;
    packet.command = command;
    if (!Send(fd_, Command, &packet, sizeof(packet)))
        throw std::runtime_error("simulation command queue full; request was not delivered");
    return packet.sequence;
}
}  // namespace qhrr05::simulation_ipc
