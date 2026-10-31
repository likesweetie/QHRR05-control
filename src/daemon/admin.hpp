#pragma once

#include "ipc/shm_utils.hpp"
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <optional>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>

namespace daemon_admin {
inline std::string Json(const std::string& value) {
    std::string result = "\"";
    for (unsigned char c : value) {
        if (c == '"' || c == '\\')
            result += '\\';
        if (c < 32)
            result += ' ';
        else
            result += char(c);
    }
    return result + '"';
}

// Local supervisor commands are independent of the process being started/stopped.
// Only the session owner can connect; clients cannot submit PIDs or executables.
class Endpoint {
public:
    struct Request {
        std::uint64_t id;
        std::string command, name, action;
    };

    explicit Endpoint(const std::string& session) {
        const auto dir = "/tmp/qhrr05-daemon-" + std::to_string(::getuid());
        if (::mkdir(dir.c_str(), 0700) && errno != EEXIST)
            shm_utils::Check(errno, "daemon admin directory");

        struct stat info {};

        if (::lstat(dir.c_str(), &info) || !S_ISDIR(info.st_mode) || info.st_uid != ::getuid() ||
            (info.st_mode & 077))
            throw std::runtime_error(
                "daemon admin directory must be owned by the user with mode 0700");
        path_ = dir + "/" + session.substr(1) + ".sock";
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (path_.size() >= sizeof(address.sun_path))
            throw std::runtime_error("daemon admin path too long");
        std::copy(path_.begin(), path_.end(), address.sun_path);
        listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (listener_ < 0)
            shm_utils::Check(errno, "daemon admin socket");
        if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address))) {
            const int error = errno;
            ::close(listener_);
            listener_ = -1;
            shm_utils::Check(error, "daemon admin bind (existing socket is not replaced)");
        }
        if (::listen(listener_, 8)) {
            const int error = errno;
            ::close(listener_);
            ::unlink(path_.c_str());
            listener_ = -1;
            shm_utils::Check(error, "daemon admin listen");
        }
    }

    ~Endpoint() {
        CloseClient();
        if (listener_ >= 0) {
            ::close(listener_);
            ::unlink(path_.c_str());
        }
    }

    std::optional<Request> Poll() {
        if (client_ < 0) {
            client_ = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (client_ < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                    return {};
                shm_utils::Check(errno, "daemon admin accept");
            }
            ucred peer{};
            socklen_t size = sizeof(peer);
            if (::getsockopt(client_, SOL_SOCKET, SO_PEERCRED, &peer, &size) ||
                peer.uid != ::getuid()) {
                CloseClient();
                return {};
            }
            opened_ = shm_utils::MonotonicNs();
        }
        char data[1024];
        const auto count = ::recv(client_, data, sizeof(data), MSG_DONTWAIT);
        if (count > 0)
            bytes_.append(data, count);
        else if (!count || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            CloseClient();
            return {};
        }
        if (bytes_.size() > 1024 || shm_utils::MonotonicNs() - opened_ > 1000000000ULL) {
            CloseClient();
            return {};
        }
        if (bytes_.find('\n') == std::string::npos)
            return {};
        std::istringstream input(bytes_);
        Request result{};
        if (!(input >> result.id >> result.command) || !result.id) {
            CloseClient();
            return {};
        }
        if (result.command == "process")
            input >> result.name >> result.action;
        else if (result.command == "stack")
            input >> result.action;
        input >> std::ws;
        if (!input.eof()) {
            Reply(result.id, false, "invalid supervisor request");
            return {};
        }
        return result;
    }

    void Reply(
        std::uint64_t id, bool ok, const std::string& message, const std::string& status = "null") {
        const auto reply = "{\"id\":" + std::to_string(id) +
                           ",\"type\":" + Json(ok ? "ack" : "error") +
                           ",\"message\":" + Json(message) + ",\"runtime\":" + status + "}\n";
        const auto sent = ::send(client_, reply.data(), reply.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
        if (sent != static_cast<ssize_t>(reply.size()))
            std::cerr << "[daemon] admin reply could not be delivered completely\n";
        CloseClient();
    }

private:
    void CloseClient() {
        if (client_ >= 0)
            ::close(client_);
        client_ = -1;
        bytes_.clear();
    }

    int listener_ = -1, client_ = -1;
    std::uint64_t opened_ = 0;
    std::string path_, bytes_;
};
}  // namespace daemon_admin
