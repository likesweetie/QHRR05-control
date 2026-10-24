#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

struct ProcessSpec {
    std::string name;
    std::string exec_path;
    std::vector<std::string> args;
    std::map<std::string, std::string> env;
    bool create_process_group{true};
};

struct ProcessInfo {
    int pid{-1};
    int pgid{-1};
    bool running{false};
    bool exited{false};
    int exit_code{0};
    int term_signal{0};
};

class Daemon {
public:
    Daemon() = default;
    ~Daemon();
    Daemon(const Daemon&) = delete;
    Daemon& operator=(const Daemon&) = delete;
    std::optional<int> Spawn(const ProcessSpec& spec);
    bool Signal(const std::string& name, int signal);
    void TerminateAll(int signal = 15);
    std::vector<std::string> Reap();
    std::optional<ProcessInfo> Get(const std::string& name) const;

    int LastErrno() const noexcept {
        return last_errno_;
    }

private:
    std::map<std::string, ProcessInfo> processes_;
    int last_errno_{0};
};
