#pragma once

#include "ipc/simulation/protocol.hpp"
#include <array>
#include <string>
#include <vector>

namespace qhrr05::simulation_ipc {
struct ApprovedCommand {
    std::uint64_t sequence = 0, issued_ns = 0, expires_ns = 0;
    SimulationCommand command{};
};

// One versioned SOCK_SEQPACKET endpoint for all simulation-only traffic.
// The physics thread never waits for subscribers. Slow readers get the latest state
// and an explicit drop counter, rather than blocking the simulation.
class Server {
public:
    explicit Server(const std::string& session);
    ~Server();
    std::vector<ApprovedCommand> Poll();
    void Publish(SimulationData state);
    void Finish();

    const SimulationData& Latest() const {
        return latest_;
    }

private:
    struct Peer {
        int fd = -1;
        int role = 0;
        bool pending = false;
        std::uint64_t dropped = 0;
    };

    void Flush(Peer& peer);
    int listener_ = -1, lock_fd_ = -1;
    std::string path_;
    std::array<Peer, 16> peers_{};
    SimulationData latest_{};
    std::uint64_t drops_ = 0;
};

class Client {
public:
    explicit Client(const std::string& session, bool controller = false);
    ~Client();
    void Poll();
    std::uint64_t SendApproved(const SimulationCommand& command);

    const SimulationData& Latest() const {
        return latest_;
    }

    bool Connected() const {
        return fd_ >= 0 && hello_;
    }

    const std::string& Status() const {
        return status_;
    }

private:
    void Close(const std::string& reason);
    int fd_ = -1;
    bool controller_ = false, hello_ = false;
    std::string path_, status_ = "waiting for simulation IPC";
    std::uint64_t retry_ns_ = 0, next_sequence_ = 0;
    SimulationData latest_{};
};
}  // namespace qhrr05::simulation_ipc
