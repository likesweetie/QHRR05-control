#include "daemon/daemon.hpp"

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

Daemon::~Daemon() {
    TerminateAll(SIGTERM);
}

int Daemon::RelayTerminalOutput(const char* descriptor_text) {
    try {
        char* end = nullptr;
        const long descriptor = std::strtol(descriptor_text, &end, 10);

        if (!*descriptor_text || *end || descriptor < 3 || descriptor > 0x7fffffff)
            throw std::runtime_error("invalid terminal descriptor");

        char buffer[4096];

        while (true) {
            const auto count = ::read(int(descriptor), buffer, sizeof(buffer));

            // Linux PTY masters report EIO when all slave descriptors close.
            if (count == 0 || (count < 0 && errno == EIO))
                return 0;

            if (count < 0 && errno == EINTR)
                continue;

            if (count < 0)
                throw std::runtime_error(
                    "terminal read failed: " + std::string(std::strerror(errno)));

            ssize_t written = 0;

            while (written < count) {
                const auto size = ::write(STDOUT_FILENO, buffer + written, count - written);

                if (size < 0 && errno == EINTR)
                    continue;

                if (size <= 0)
                    throw std::runtime_error("terminal output failed");

                written += size;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "[daemon] " << error.what() << '\n';
        return 1;
    }
}

std::optional<int> Daemon::Spawn(const ProcessSpec& spec) {
    last_errno_ = 0;

    if (spec.name.empty() || spec.exec_path.empty() || processes_.count(spec.name)) {
        last_errno_ = EINVAL;
        return std::nullopt;
    }

    int acknowledgement[2];

    if (::pipe2(acknowledgement, O_CLOEXEC) != 0) {
        last_errno_ = errno;
        return std::nullopt;
    }

    int ready[2] = {-1, -1};

    if (spec.wait_ready && ::pipe2(ready, O_CLOEXEC) != 0) {
        last_errno_ = errno;
        ::close(acknowledgement[0]);
        ::close(acknowledgement[1]);
        return std::nullopt;
    }

    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(spec.exec_path.c_str()));

    for (const auto& argument : spec.args)
        argv.push_back(const_cast<char*>(argument.c_str()));

    argv.push_back(nullptr);
    const pid_t pid = ::fork();

    if (pid < 0) {
        last_errno_ = errno;
        ::close(acknowledgement[0]);
        ::close(acknowledgement[1]);
        if (spec.wait_ready) {
            ::close(ready[0]);
            ::close(ready[1]);
        }

        return std::nullopt;
    }

    if (pid == 0) {
        ::close(acknowledgement[0]);
        const auto fail = [&] {
            const int reason = errno;
            (void)::write(acknowledgement[1], &reason, sizeof(reason));
            _exit(127);
        };

        if (spec.wait_ready) {
            ::close(ready[0]);

            if (::fcntl(ready[1], F_SETFD, 0) < 0 ||
                ::setenv("QHRR_PROCESS_READY_FD", std::to_string(ready[1]).c_str(), 1) != 0)
                fail();
        }

        if (!spec.working_directory.empty() && ::chdir(spec.working_directory.c_str()) != 0)
            fail();

        if (spec.create_process_group && ::setpgid(0, 0) != 0)
            fail();

        if (spec.output_fd >= 0 && (::dup2(spec.output_fd, STDOUT_FILENO) < 0 ||
                                       ::dup2(spec.output_fd, STDERR_FILENO) < 0))
            fail();

        if (spec.pass_fd >= 0 && ::fcntl(spec.pass_fd, F_SETFD, 0) < 0)
            fail();

        for (const auto& [key, value] : spec.env)
            if (::setenv(key.c_str(), value.c_str(), 1) != 0)
                fail();

        ::execv(spec.exec_path.c_str(), argv.data());
        fail();
    }

    ::close(acknowledgement[1]);
    if (spec.wait_ready)
        ::close(ready[1]);
    int exec_error = 0;
    ssize_t read_count;
    do {
        read_count = ::read(acknowledgement[0], &exec_error, sizeof(exec_error));
    } while (read_count < 0 && errno == EINTR);
    const int read_error = errno;
    ::close(acknowledgement[0]);

    bool ready_ok = true;
    int ready_error = 0;

    if (spec.wait_ready) {
        pollfd descriptor{ready[0], POLLIN, 0};
        char value = 0;
        int result;

        do {
            result = ::poll(&descriptor, 1, spec.ready_timeout_ms);
        } while (result < 0 && errno == EINTR);

        ready_ok = result > 0 && ::read(ready[0], &value, 1) == 1 && value == 'R';
        ready_error = result == 0 ? ETIMEDOUT : (result < 0 ? errno : EPIPE);
        ::close(ready[0]);
    }

    if (read_count != 0 || !ready_ok) {
        last_errno_ = read_count > 0 ? exec_error : read_error;

        if (read_count == 0)
            last_errno_ = ready_error;

        ::kill(spec.create_process_group ? -pid : pid, SIGKILL);

        while (::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
        }

        return std::nullopt;
    }

    ProcessInfo info;
    info.pid = pid;
    info.pgid = spec.create_process_group ? pid : -1;
    info.running = true;
    processes_[spec.name] = info;

    return pid;
}

bool Daemon::Signal(const std::string& name, int signal) {
    const auto found = processes_.find(name);

    if (found == processes_.end() || !found->second.running)
        return false;

    const auto& process = found->second;
    const int target = process.pgid > 0 ? -process.pgid : process.pid;

    if (::kill(target, signal) == 0)
        return true;

    last_errno_ = errno;

    return false;
}

void Daemon::TerminateAll(int signal) {
    for (const auto& [name, _] : processes_)
        Signal(name, signal);
}

std::vector<std::string> Daemon::Reap() {
    std::vector<std::string> exited;

    for (auto& [name, process] : processes_) {
        if (!process.running)
            continue;

        int status = 0;
        const pid_t result = ::waitpid(process.pid, &status, WNOHANG);

        if (result == 0 || (result < 0 && errno == EINTR))
            continue;

        process.running = false;
        process.exited = true;

        if (result < 0) {
            process.exit_code = 1;
        } else if (WIFEXITED(status)) {
            process.exit_code = WEXITSTATUS(status);
        } else if (WIFSIGNALED(status)) {
            process.term_signal = WTERMSIG(status);
        }

        exited.push_back(name);
    }

    return exited;
}

std::optional<ProcessInfo> Daemon::Get(const std::string& name) const {
    const auto found = processes_.find(name);

    if (found == processes_.end())
        return std::nullopt;

    return found->second;
}
