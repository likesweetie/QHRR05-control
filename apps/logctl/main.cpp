#include "shm_logger/logger_service.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <iostream>
#include <cstring>

int main(int argc, char** argv) {
    int fd = -1;
    try {
        std::string socket_path, request;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--socket" && i + 1 < argc)
                socket_path = argv[++i];
            else {
                if (!request.empty())
                    request += ' ';
                request += argument;
            }
        }
        if (request.empty())
            throw std::runtime_error(
                "usage: qhrr-logctl [--socket PATH] status|start|stop|trigger|mark TEXT|sessions|health");
        if (socket_path.empty()) {
            const auto config = YAML::LoadFile(QHRR_DEFAULT_DAEMON_CONFIG);
            socket_path = qhrr05::logging::ControlSocketPath(config["shm_name"].as<std::string>());
        }
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (socket_path.size() >= sizeof(address.sun_path))
            throw std::runtime_error("socket path too long");
        std::strcpy(address.sun_path, socket_path.c_str());
        fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (fd < 0)
            throw std::runtime_error(std::strerror(errno));
        const timeval timeout{5, 0};
        if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0 ||
            ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0)
            throw std::runtime_error("cannot set command timeout");
        if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
            throw std::runtime_error("logger connect " + socket_path + ": " + std::strerror(errno));
        if (request.size() > 4096 || ::send(fd, request.data(), request.size(), MSG_NOSIGNAL) !=
                                         static_cast<ssize_t>(request.size()))
            throw std::runtime_error("logger command send failed");
        std::array<char, 65536> response{};
        const auto count = ::recv(fd, response.data(), response.size(), MSG_TRUNC);
        if (count <= 0 || count >= static_cast<ssize_t>(response.size()))
            throw std::runtime_error("logger response failed/truncated");
        ::close(fd);
        fd = -1;
        const std::string text(response.data(), count);
        std::cout << text << '\n';
        return YAML::Load(text)["ok"].as<bool>() ? 0 : 2;
    } catch (const std::exception& e) {
        if (fd >= 0)
            ::close(fd);
        std::cerr << "[qhrr-logctl] ERROR: " << e.what() << '\n';
        return 1;
    }
}
