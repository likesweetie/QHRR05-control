#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>
#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>

struct ProcessSpec {
    std::string name;
    std::string exec_path;
    std::vector<std::string> args;
    std::map<std::string, std::string> env;
    std::string working_directory;
    std::string terminal_title;
    std::string terminal_exec;
    int output_fd{-1};
    int pass_fd{-1};
    bool wait_ready{false};
    int ready_timeout_ms{5000};
    bool create_process_group{true};
    bool terminal_viewer{false};
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

    // Statistics frames are separated from log output by the terminal relay.
    static constexpr const char* kStatisticsBegin = "\033]777;qhrr-statistics;";
    static constexpr char kStatisticsEnd = '\a';
    static int RelayTerminalOutput(const char* descriptor_text,
        const char* pid_text,
        const char* process_name,
        const char* interval_text,
        const char* log_path);
    void DisplayStatistics(const std::string& text);
    void ResetStatistics();

    // One startup notification from the child, not a runtime response check.
    static void NotifyReady() {
        const char* value = std::getenv("QHRR_PROCESS_READY_FD");

        if (!value)
            return;

        char* end = nullptr;
        const long descriptor = std::strtol(value, &end, 10);

        if (!*value || *end || descriptor < 3 || descriptor > 0x7fffffff)
            throw std::runtime_error("invalid process readiness descriptor");

        const char ready = 'R';
        ssize_t count;

        do {
            count = ::write(int(descriptor), &ready, 1);
        } while (count < 0 && errno == EINTR);

        ::close(int(descriptor));
        ::unsetenv("QHRR_PROCESS_READY_FD");

        if (count != 1)
            throw std::runtime_error("process readiness notification failed");
    }

    int LastErrno() const noexcept {
        return last_errno_;
    }

private:
    std::map<std::string, ProcessInfo> processes_;
    int last_errno_{0};
    int statistics_rows_{0}, terminal_rows_{0}, terminal_columns_{0};
};
