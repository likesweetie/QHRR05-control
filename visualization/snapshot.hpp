#pragma once

#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <deque>
#include <vector>

namespace qhrr05::visualization {
struct Snapshot {
    SimulationData simulation{};
    MotorData motor{};
    ModelData model{};
    ControlData control{};
    std::uint64_t sequence = 0;
    bool stopping = false, failed = false, finished = false;
    std::string simulation_status;
};

// The observer never writes commands or simulation state. Only pthread mutex bookkeeping
// needs a writable mapping. GPU work never executes on this thread or under the SHM lock.
class SnapshotSource {
public:
    explicit SnapshotSource(const std::string& name);
    ~SnapshotSource();
    Snapshot Latest();
    std::vector<Snapshot> Drain();

    std::uint64_t Dropped() const {
        return dropped_;
    }

    std::uint64_t Reads() const {
        return reads_;
    }

    std::uint64_t Misses() const {
        return misses_;
    }

    std::uint64_t MaxHoldNs() const {
        return max_hold_ns_;
    }

private:
    void Observe(std::stop_token stop);
    shm_utils::Mapping mapping_;
    simulation_ipc::Client simulation_channel_;
    std::mutex mailbox_mutex_;
    Snapshot latest_{};
    std::deque<Snapshot> pending_;
    std::exception_ptr error_;
    std::atomic<std::uint64_t> reads_{0}, misses_{0}, max_hold_ns_{0};
    std::atomic<std::uint64_t> dropped_{0};
    std::jthread worker_;
};
}  // namespace qhrr05::visualization
