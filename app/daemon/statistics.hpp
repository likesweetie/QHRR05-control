#pragma once

#include "shared_memory.hpp"
#include <array>
#include <deque>
#include <string>
#include <utility>

struct WindowSummary {
    std::size_t count = 0;
    double minimum = 0, maximum = 0, mean = 0, standard_deviation = 0;
};

class WindowStatistics {
public:
    void Add(std::uint64_t time_ns, double value);
    WindowSummary Read(std::uint64_t now_ns, std::uint64_t window_ns);

private:
    std::deque<std::pair<std::uint64_t, double>> values_;
};

// Read-only observer. No instrumentation or extra fields are added to the IPC contract.
class ShmStatistics {
public:
    ShmStatistics(std::string name, double window_sec);
    void Sample(ShmData& shm);
    std::string Report(std::uint64_t now_ns);

private:
    std::string name_;
    std::uint64_t window_ns_;
    std::array<WindowStatistics, 4> ages_, gaps_, intervals_;
    WindowStatistics model_to_control_, encoder_age_, unconsumed_commands_;
    std::array<std::uint64_t, 4> previous_sequence_{}, previous_publication_{};
    std::uint64_t previous_control_sequence_ = 0;
    std::uint64_t samples_ = 0, busy_ = 0, regressions_ = 0, invalid_times_ = 0;
    bool stopping_ = false, failed_ = false, can_enabled_ = false;
};
