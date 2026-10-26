#include "daemon/daemon.hpp"

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <clocale>
#include <cwchar>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/wait.h>
#include <unistd.h>

Daemon::~Daemon() {
    ResetStatistics();
    TerminateAll(SIGTERM);
}

namespace {
void WriteOutput(const std::string& text) {
    std::size_t offset = 0;

    while (offset < text.size()) {
        const auto size = ::write(STDOUT_FILENO, text.data() + offset, text.size() - offset);

        if (size < 0 && errno == EINTR)
            continue;

        if (size <= 0)
            throw std::runtime_error("terminal output failed");

        offset += size;
    }
}
}

namespace {
std::string FitTerminalLine(const std::string& line, int columns, bool& clipped) {
    std::string output;
    std::mbstate_t state{};
    int width = 0;

    for (std::size_t i = 0; i < line.size();) {
        if (line[i] == '\033' && i + 1 < line.size() && line[i + 1] == '[') {
            const auto end = line.find_first_of(
                "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~", i + 2);
            if (end == std::string::npos)
                break;
            if (line[end] == 'm')
                output.append(line, i, end - i + 1);
            i = end + 1;
            continue;
        }

        wchar_t character = 0;
        auto length = std::mbrtowc(&character, line.data() + i, line.size() - i, &state);
        if (length == std::size_t(-1) || length == std::size_t(-2) || length == 0) {
            character = L'?';
            length = 1;
            state = {};
        }
        const int cells = std::max(0, ::wcwidth(character));
        if (width + cells > columns) {
            clipped = true;
            break;
        }
        if (character >= L' ')
            output.append(line, i, length);
        width += cells;
        i += length;
    }
    return output + "\033[0m";
}

volatile std::sig_atomic_t relay_signal = 0;

void OnRelaySignal(int signal) {
    relay_signal = signal;
}

class RelaySignals {
public:
    RelaySignals() {
        relay_signal = 0;

        struct sigaction action {};

        action.sa_handler = OnRelaySignal;
        ::sigemptyset(&action.sa_mask);
        for (int i = 0; i < 3; ++i) {
            if (::sigaction(signals_[i], &action, &previous_[i]) != 0) {
                for (int j = 0; j < i; ++j)
                    ::sigaction(signals_[j], &previous_[j], nullptr);
                throw std::runtime_error("cannot install terminal shutdown handlers");
            }
        }
    }

    ~RelaySignals() {
        for (int i = 0; i < 3; ++i)
            ::sigaction(signals_[i], &previous_[i], nullptr);
    }

private:
    int signals_[3]{SIGINT, SIGTERM, SIGHUP};
    struct sigaction previous_[3]{};
};
}

void Daemon::BeginTerminalScreen() {
    if (terminal_screen_active_ || !::isatty(STDOUT_FILENO))
        return;
    if (!std::setlocale(LC_CTYPE, ""))
        throw std::runtime_error("cannot initialize terminal character locale");
    // Save the title and normal screen before layout/logging writes anything.
    terminal_screen_active_ = true;
    WriteOutput("\033[22;0t\033[?1049h\033[?25l\033[0m\033[r\033[H\033[2J");
}

std::string Daemon::TerminalLogPane() const {
    if (statistics_rows_ <= 0)
        return {};
    std::vector<std::string> lines(terminal_log_lines_.begin(), terminal_log_lines_.end());
    if (!terminal_log_fragment_.empty())
        lines.push_back(terminal_log_fragment_);
    const int count = terminal_rows_ - statistics_rows_;
    const auto begin = lines.size() > std::size_t(count) ? lines.size() - count : 0;
    std::ostringstream output;
    for (int row = 0; row < count; ++row) {
        output << "\033[" << statistics_rows_ + row + 1 << ";1H\033[2K";
        if (begin + row < lines.size()) {
            bool clipped = false;
            output << FitTerminalLine(lines[begin + row], terminal_columns_ - 1, clipped);
        }
    }
    return output.str();
}

void Daemon::WriteTerminalLog(const std::string& text) {
    if (!::isatty(STDOUT_FILENO)) {
        WriteOutput(text);
        return;
    }
    BeginTerminalScreen();
    for (char character : text) {
        if (character == '\n') {
            terminal_log_lines_.push_back(std::move(terminal_log_fragment_));
            terminal_log_fragment_.clear();
            if (terminal_log_lines_.size() > 200)
                terminal_log_lines_.pop_front();
        } else if (character == '\r')
            terminal_log_fragment_.clear();
        else
            terminal_log_fragment_ += character;
        if (terminal_log_fragment_.size() > 8192)
            terminal_log_fragment_ =
                "[long line: full text in log file] " + terminal_log_fragment_.substr(4096);
    }
    WriteOutput(statistics_rows_ > 0 ? TerminalLogPane() : text);
}

void Daemon::DisplayStatistics(const std::string& text) {
    if (!::isatty(STDOUT_FILENO)) {
        WriteOutput(text);
        return;
    }
    BeginTerminalScreen();
    winsize size{};
    if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) != 0)
        throw std::runtime_error("cannot read statistics terminal size");
    if (size.ws_row < 6 || size.ws_col < 20) {
        if (statistics_rows_ != -1) {
            WriteOutput(
                "\033[0m\033[r\033[H\033[2J[statistics] Waiting for terminal size >=20x6; logs remain active.\n");
            statistics_rows_ = -1;
        }
        return;
    }
    std::vector<std::string> lines;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line))
        lines.push_back(line);
    const int rows = std::min<int>(lines.size() + 1, size.ws_row - 4);
    std::ostringstream output;
    if (statistics_rows_ != rows || terminal_rows_ != size.ws_row ||
        terminal_columns_ != size.ws_col) {
        // Clear only the alternate screen, then replay recent logs at the new size.
        output << "\033[0m\033[r\033[H\033[2J";
        statistics_rows_ = rows;
        terminal_rows_ = size.ws_row;
        terminal_columns_ = size.ws_col;
    }
    bool clipped = int(lines.size()) >= rows;
    for (int i = 0; i < rows; ++i) {
        output << "\033[" << i + 1 << ";1H\033[2K";
        if (i < rows - 1 && i < int(lines.size()))
            output << FitTerminalLine(lines[i], size.ws_col - 1, clipped);
        else if (i == rows - 1) {
            const std::string separator =
                clipped ? "--- LOGS | statistics clipped: enlarge terminal ---"
                        : "--- LOGS | recent messages; full history in log files ---";
            bool ignored = false;
            output << "\033[2;36m" << FitTerminalLine(separator, size.ws_col - 1, ignored);
        }
    }
    output << TerminalLogPane();
    WriteOutput(output.str());
}

void Daemon::ResetStatistics() noexcept {
    if (!terminal_screen_active_)
        return;
    terminal_screen_active_ = false;
    statistics_rows_ = terminal_rows_ = terminal_columns_ = 0;
    const char reset[] = "\033[0m\033[r\033[?25h\033[?1049l\033[23;0t";
    std::size_t offset = 0;
    while (offset < sizeof(reset) - 1) {
        const auto count = ::write(STDOUT_FILENO, reset + offset, sizeof(reset) - 1 - offset);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
            break;  // A closed terminal cannot be restored; destructors cannot throw.
        offset += count;
    }
}

int Daemon::RelayTerminalOutput(const char* descriptor_text,
    const char* pid_text,
    const char* process_name,
    const char* interval_text,
    const char* log_path,
    const char* supervisor_text) {
    try {
        char* end = nullptr;
        const long descriptor = std::strtol(descriptor_text, &end, 10);

        if (!*descriptor_text || *end || descriptor < 3 || descriptor > 0x7fffffff)
            throw std::runtime_error("invalid terminal descriptor");

        const long pid = std::strtol(pid_text, &end, 10);

        if (!*pid_text || *end || pid <= 0 || pid > 0x7fffffff)
            throw std::runtime_error("invalid terminal process PID");

        const long supervisor = std::strtol(supervisor_text, &end, 10);
        if (!*supervisor_text || *end || supervisor <= 0 || supervisor > 0x7fffffff)
            throw std::runtime_error("invalid terminal supervisor PID");
        RelaySignals signals;

        const double interval_sec = std::strtod(interval_text, &end);

        if (!*interval_text || *end || !std::isfinite(interval_sec) || interval_sec <= 1 ||
            interval_sec > 60)
            throw std::runtime_error("terminal statistics interval must be in (1, 60] seconds");

        const long ticks_per_second = ::sysconf(_SC_CLK_TCK);
        const long page_size = ::sysconf(_SC_PAGESIZE);

        if (ticks_per_second <= 0 || page_size <= 0)
            throw std::runtime_error("cannot read process statistics units");

        Daemon display;
        display.BeginTerminalScreen();
        bool shutdown_requested = false;
        std::ofstream log(log_path, std::ios::app);

        if (!log)
            throw std::runtime_error("cannot open process log: " + std::string(log_path));

        const auto write_log = [&](const std::string& text) {
            if (text.empty())
                return;

            log << text << std::flush;

            if (!log)
                throw std::runtime_error("process log write failed: " + std::string(log_path));

            display.WriteTerminalLog(text);
        };
        const auto period = std::chrono::duration<double>(interval_sec);
        auto next_display = std::chrono::steady_clock::now();
        auto previous_time = next_display;
        std::uint64_t previous_cpu_ticks = 0;
        bool have_cpu_sample = false;
        std::string pending, application_statistics;
        const std::string marker = kStatisticsBegin;
        bool in_statistics = false;
        char buffer[4096];

        while (true) {
            if (relay_signal && !shutdown_requested) {
                shutdown_requested = true;
                if (::kill(int(supervisor), relay_signal == SIGINT ? SIGINT : SIGTERM) != 0) {
                    if (errno == ESRCH)
                        return 0;
                    throw std::runtime_error("cannot notify daemon of terminal shutdown");
                }
            }
            const auto now = std::chrono::steady_clock::now();

            if (now >= next_display) {
                std::ifstream status("/proc/" + std::to_string(pid) + "/stat");
                std::string record;
                std::getline(status, record);
                const auto closing = record.rfind(')');
                std::ostringstream text;
                text << "\033[1;36m" << process_name << " | PID " << pid << " | refresh "
                     << interval_sec << " s\033[0m\n";

                if (closing == std::string::npos) {
                    if (::kill(pid, 0) == 0 || errno != ESRCH)
                        throw std::runtime_error(
                            "cannot read process statistics: " + std::to_string(pid));

                    text << "Process exited; draining remaining output.\n";
                } else {
                    std::istringstream fields(record.substr(closing + 2));
                    std::vector<std::string> values;
                    std::string value;

                    while (fields >> value)
                        values.push_back(value);

                    if (values.size() < 22)
                        throw std::runtime_error("invalid /proc process statistics");

                    const auto cpu_ticks = std::stoull(values[11]) + std::stoull(values[12]);
                    const double elapsed =
                        std::chrono::duration<double>(now - previous_time).count();
                    timespec boot_time{};

                    if (::clock_gettime(CLOCK_BOOTTIME, &boot_time) != 0)
                        throw std::runtime_error("cannot read process uptime clock");

                    const double uptime = boot_time.tv_sec + boot_time.tv_nsec / 1e9 -
                                          std::stoull(values[19]) / double(ticks_per_second);
                    text << "state=" << values[0] << " CPU=";

                    if (have_cpu_sample && elapsed > 0 && cpu_ticks >= previous_cpu_ticks)
                        text << std::fixed << std::setprecision(1)
                             << 100.0 * (cpu_ticks - previous_cpu_ticks) /
                                    (ticks_per_second * elapsed)
                             << '%';
                    else
                        text << "N/A (first sample)";

                    if (have_cpu_sample && elapsed > 0 && cpu_ticks >= previous_cpu_ticks) {
                        const double cpu_percent =
                            100.0 * (cpu_ticks - previous_cpu_ticks) / (ticks_per_second * elapsed);
                        const int filled =
                            int(std::round(std::clamp(cpu_percent, 0.0, 100.0) * .12));
                        text << " [" << std::string(filled, '#') << std::string(12 - filled, '.')
                             << ']';
                    }
                    text << " RSS=" << std::fixed << std::setprecision(1)
                         << std::stoll(values[21]) * double(page_size) / (1024 * 1024)
                         << " MiB threads=" << values[17] << " uptime=" << uptime << " s\n";
                    previous_cpu_ticks = cpu_ticks;
                    previous_time = now;
                    have_cpu_sample = true;
                }

                text << "Log: " << log_path << '\n' << application_statistics;
                // Keep the panel height stable before the first HAL statistics frame arrives.
                std::string report = text.str();

                while (std::count(report.begin(), report.end(), '\n') < 12)
                    report += '\n';

                display.DisplayStatistics(report);
                next_display =
                    now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            }

            const auto remaining = next_display - std::chrono::steady_clock::now();
            const int timeout_ms = std::max(
                0, int(std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count()));
            pollfd source{int(descriptor), POLLIN, 0};
            const int ready = ::poll(&source, 1, timeout_ms);

            if (ready < 0 && errno == EINTR)
                continue;

            if (ready < 0)
                throw std::runtime_error("terminal relay poll failed");

            if (ready == 0)
                continue;

            const auto count = ::read(int(descriptor), buffer, sizeof(buffer));

            // Linux PTY masters report EIO when all slave descriptors close.
            if (count == 0 || (count < 0 && errno == EIO)) {
                if (in_statistics)
                    throw std::runtime_error("incomplete terminal statistics frame");

                write_log(pending);
                return 0;
            }

            if (count < 0 && errno == EINTR)
                continue;

            if (count < 0)
                throw std::runtime_error(
                    "terminal read failed: " + std::string(std::strerror(errno)));

            pending.append(buffer, count);

            for (;;) {
                if (in_statistics) {
                    const auto end = pending.find(kStatisticsEnd);

                    if (end == std::string::npos) {
                        if (pending.size() > 16384)
                            throw std::runtime_error("terminal statistics frame too large");

                        break;
                    }

                    application_statistics = pending.substr(0, end);
                    pending.erase(0, end + 1);
                    in_statistics = false;
                } else {
                    const auto start = pending.find(marker);

                    if (start != std::string::npos) {
                        write_log(pending.substr(0, start));
                        pending.erase(0, start + marker.size());
                        in_statistics = true;
                    } else {
                        std::size_t keep = std::min(pending.size(), marker.size() - 1);

                        while (keep &&
                               pending.compare(pending.size() - keep, keep, marker, 0, keep) != 0)
                            --keep;

                        write_log(pending.substr(0, pending.size() - keep));
                        pending.erase(0, pending.size() - keep);
                        break;
                    }
                }
            }
        }
    } catch (const std::exception& error) {
        std::ofstream log(log_path, std::ios::app);
        log << "[terminal relay] " << error.what() << '\n';
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
