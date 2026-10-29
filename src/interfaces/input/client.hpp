#pragma once

#include "ipc/simulation/protocol.hpp"

#include "ipc/shared_memory.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace qhrr05 {
struct InputDecision {
    std::uint64_t id = 0, sequence = 0;
    bool approved = false, unknown = false;
    std::string operation, message;
};

// Asynchronous Input socket producer. Has no SHM mapping or command write path.
class InputClient {
public:
    explicit InputClient(const std::string& shm_name);
    ~InputClient();
    std::uint64_t Submit(const std::string& operation);
    void SetPerturb(PerturbTarget target);
    void ReleasePerturb();
    std::vector<InputDecision> Decisions();

    std::uint64_t owner() const {
        return owner_;
    }

private:
    struct Request {
        std::uint64_t id = 0;
        std::string operation;
        bool heartbeat = false;
    };

    void Run();
    InputDecision Exchange(const Request& request);
    std::string PerturbLine(const PerturbTarget& target) const;
    std::string path_;
    std::uint64_t owner_;
    std::atomic<bool> stopping_{false};
    std::uint64_t next_id_ = 0, perturb_updated_ns_ = 0;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Request> queue_;
    std::vector<InputDecision> decisions_;
    PerturbTarget perturb_{};
    bool perturb_reported_ = false;
    std::thread worker_;
};
}  // namespace qhrr05
