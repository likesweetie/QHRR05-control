#pragma once

#include "ipc/shared_memory.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <string>

namespace qhrr05::logging {

// Local copy only. No synchronization objects or references into SHM are serialized.
struct LogFrame {
    std::uint64_t capture_ns = 0, lock_acquired_ns = 0, lock_released_ns = 0;
    std::uint64_t motor_sequence = 0, model_sequence = 0, control_sequence = 0,
                  simulation_sequence = 0;
    std::uint32_t abi_version = kShmAbiVersion;
    bool stopping = false, failed = false, simulation_finished = false, can_enabled = false;
    bool simulation_valid = false;
    MotorData motor{};
    ModelData model{};
    ControlData control{};
    SimulationData simulation{};
};

struct ReaderCounters {
    std::uint64_t attempts = 0, successes = 0, lock_misses = 0, recoveries = 0, failures = 0;
    double wait_average_us = 0, wait_max_us = 0, hold_average_us = 0, hold_max_us = 0;
};

class SnapshotReader {
public:
    explicit SnapshotReader(const std::string& name);
    ~SnapshotReader();
    SnapshotReader(const SnapshotReader&) = delete;
    SnapshotReader& operator=(const SnapshotReader&) = delete;
    bool Read(LogFrame& frame);
    bool SameObject() const;
    ReaderCounters Counters() const;
    void ResetCounters();  // Only while the sampler is stopped.

private:
    std::string name_;
    ShmData* shm_ = nullptr;
    int fd_ = -1;
    std::uint64_t device_ = 0, inode_ = 0;
    std::atomic<std::uint64_t> attempts_{0}, successes_{0}, misses_{0}, recoveries_{0},
        failures_{0}, wait_ns_{0}, wait_max_ns_{0}, hold_ns_{0}, hold_max_ns_{0};
};
}  // namespace qhrr05::logging
