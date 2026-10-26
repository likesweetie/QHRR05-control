#include "daemon/daemon.hpp"
#include "shm_utils.hpp"
#include "statistics.hpp"

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace {
namespace fs = std::filesystem;
volatile std::sig_atomic_t stop_requested = 0;

void OnSignal(int) {
    stop_requested = 1;
}

fs::path ExecutableDirectory() {
    char buffer[4096];
    const auto size = ::readlink("/proc/self/exe", buffer, sizeof(buffer));

    if (size <= 0 || size == sizeof(buffer))
        throw std::runtime_error("cannot resolve daemon executable directory");

    return fs::path(std::string(buffer, size)).parent_path();
}

std::vector<ProcessSpec> LoadProcesses(
    const fs::path& path, const YAML::Node& config, bool without_can, bool no_terminals) {
    const std::string section = without_can ? "debug_processes" : "processes";
    const auto processes = config[section];

    if (!config.IsMap() || !processes || !processes.IsSequence() || processes.size() == 0)
        throw std::runtime_error("daemon config requires a nonempty " + section + " sequence");

    if (!config["shm_name"] || !config["shm_name"].IsScalar())
        throw std::runtime_error("daemon config requires shm_name");

    const auto shm_name = config["shm_name"].as<std::string>();

    if (shm_name.size() < 2 || shm_name.front() != '/' ||
        shm_name.find('/', 1) != std::string::npos)
        throw std::runtime_error("daemon shm_name must be / followed by a single name");

    std::vector<ProcessSpec> result;

    for (const auto& process : processes) {
        ProcessSpec spec;
        spec.name = process["name"].as<std::string>();
        spec.exec_path =
            fs::absolute(ExecutableDirectory() / process["exec_path"].as<std::string>()).string();
        spec.args = process["args"].as<std::vector<std::string>>();

        for (auto& argument : spec.args)
            if (argument == "{shm_name}")
                argument = shm_name;

        if (spec.name.empty() || !fs::is_regular_file(spec.exec_path))
            throw std::runtime_error("invalid process name or executable: " + spec.name);

        if (spec.name.find_first_not_of(
                "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") !=
                std::string::npos ||
            spec.name == "." || spec.name == "..")
            throw std::runtime_error(
                "process name must be a single safe log filename: " + spec.name);

        for (const auto& existing : result)
            if (existing.name == spec.name)
                throw std::runtime_error("duplicate process name: " + spec.name);

        if (process["working_directory"])
            spec.working_directory =
                fs::absolute(path.parent_path() / process["working_directory"].as<std::string>())
                    .string();

        if (process["wait_ready"])
            spec.wait_ready = process["wait_ready"].as<bool>();

        if (!no_terminals) {
            spec.terminal_title = process["terminal_title"]
                                      ? process["terminal_title"].as<std::string>()
                                      : "QHRR05-" + spec.name;
            const auto terminal =
                process["terminal_exec"] ? process["terminal_exec"] : config["terminal_exec"];
            spec.terminal_exec =
                fs::absolute(ExecutableDirectory() / terminal.as<std::string>()).string();

            if (spec.terminal_title.empty() || !fs::is_regular_file(spec.terminal_exec))
                throw std::runtime_error("invalid terminal configuration: " + spec.name);
        }

        if (process["ready_timeout_ms"])
            spec.ready_timeout_ms = process["ready_timeout_ms"].as<int>();

        if (spec.ready_timeout_ms <= 0)
            throw std::runtime_error("ready_timeout_ms must be positive");

        if (process["env"])
            for (const auto& entry : process["env"])
                spec.env.emplace(entry.first.as<std::string>(), entry.second.as<std::string>());

        result.push_back(std::move(spec));
    }

    for (const auto& spec : result)
        if (!spec.terminal_title.empty())
            for (const auto& other : result)
                if (other.name == spec.name + ".terminal")
                    throw std::runtime_error("reserved terminal process name: " + other.name);

    return result;
}

// The application remains a direct child. The terminal only displays its PTY output.
struct ProcessTerminal {
    int master{-1};
    int slave{-1};

    ~ProcessTerminal() {
        if (slave >= 0)
            ::close(slave);

        if (master >= 0)
            ::close(master);
    }

    void Open() {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);

        if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0)
            throw std::runtime_error(
                "cannot create process terminal: " + std::string(std::strerror(errno)));

        char* path = ::ptsname(master);

        if (!path)
            throw std::runtime_error("cannot resolve process terminal");

        slave = ::open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
        termios attributes{};

        if (slave < 0 || ::tcgetattr(slave, &attributes) != 0)
            throw std::runtime_error("cannot open process terminal");

        ::cfmakeraw(&attributes);

        if (::tcsetattr(slave, TCSANOW, &attributes) != 0)
            throw std::runtime_error("cannot configure process terminal");
    }

    void ReportStartupError(const fs::path& log_path) {
        if (::fcntl(master, F_SETFL, O_NONBLOCK) < 0)
            throw std::runtime_error("cannot read process startup output");

        char buffer[4096];
        std::ofstream log(log_path, std::ios::app);

        if (!log)
            throw std::runtime_error("cannot open startup log: " + log_path.string());

        while (true) {
            const auto count = ::read(master, buffer, sizeof(buffer));

            if (count > 0) {
                std::cerr.write(buffer, count);
                log.write(buffer, count);

                if (!log)
                    throw std::runtime_error("cannot write startup log: " + log_path.string());
            } else if (count < 0 && errno == EINTR)
                continue;
            else if (count == 0 || errno == EAGAIN || errno == EIO)
                break;
            else
                throw std::runtime_error("cannot read process startup output");
        }
    }
};

void StartProcess(Daemon& daemon, const ProcessSpec& spec, std::vector<ProcessSpec>& children) {
    const auto pid = daemon.Spawn(spec);

    if (!pid)
        throw std::runtime_error(
            "process start failed: " + spec.name + ": " + std::strerror(daemon.LastErrno()));

    children.push_back(spec);
    std::cout << "[daemon] " << spec.name << " started pid=" << *pid << std::endl;
}

bool AnyRunning(Daemon& daemon, const std::vector<ProcessSpec>& specs) {
    bool running = false;

    for (const auto& spec : specs) {
        const auto info = daemon.Get(spec.name);
        running |= info && info->running;
    }

    return running;
}

bool StopChildren(Daemon& daemon, const std::vector<ProcessSpec>& specs) {
    // Relays must drain EOF (including shutdown messages) before their windows close.
    for (const auto& spec : specs)
        if (!spec.terminal_viewer)
            daemon.Signal(spec.name, SIGTERM);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (AnyRunning(daemon, specs) && std::chrono::steady_clock::now() < deadline) {
        daemon.Reap();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool forced = false;

    for (const auto& spec : specs) {
        const auto info = daemon.Get(spec.name);

        if (!spec.terminal_viewer && info && info->running) {
            forced = true;
            daemon.Signal(spec.name, SIGKILL);
        }
    }

    const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    while (AnyRunning(daemon, specs) && std::chrono::steady_clock::now() < drain_deadline) {
        daemon.Reap();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    forced |= AnyRunning(daemon, specs);
    daemon.TerminateAll(SIGKILL);

    while (AnyRunning(daemon, specs)) {
        daemon.Reap();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return forced;
}
}  // namespace

int main(int argc, char** argv) {
    // Display-only mode must retain the terminal's normal signal handling.
    if (argc == 7 && std::string(argv[1]) == "--terminal-output")
        return Daemon::RelayTerminalOutput(argv[2], argv[3], argv[4], argv[5], argv[6]);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGHUP, OnSignal);
    Daemon daemon;
    std::vector<ProcessSpec> children;
    std::vector<std::unique_ptr<ProcessTerminal>> terminals;
    // Keep the shared region alive until every child has been stopped, including startup failures.
    std::unique_ptr<shm_utils::Mapping> shared_memory;

    try {
        fs::path config_path = QHRR05_DEFAULT_DAEMON_CONFIG;
        bool without_can = false;
        bool no_terminals = false;
        const char* usage =
            "usage: daemon [--config PROCESS_CONFIG.yaml] [--without-can] [--no-terminals]";

        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];

            if (argument == "--without-can")
                without_can = true;
            else if (argument == "--no-terminals")
                no_terminals = true;
            else if (argument == "--config" && i + 1 < argc)
                config_path = argv[++i];
            else if (argument == "--help") {
                std::cout << usage << '\n';
                return 0;
            } else
                throw std::runtime_error(usage);
        }

        config_path = fs::absolute(config_path);
        const auto config = YAML::LoadFile(config_path.string());
        const auto specs = LoadProcesses(config_path, config, without_can, no_terminals);
        const auto shm_name = config["shm_name"].as<std::string>();
        const auto monitoring = config["monitoring"];
        const double display_interval = monitoring["display_interval_sec"].as<double>();
        const double sample_hz = monitoring["sample_hz"].as<double>();

        if (!std::isfinite(display_interval) || display_interval <= 1 || display_interval > 60)
            throw std::runtime_error("monitoring.display_interval_sec must be in (1, 60]");

        if (!std::isfinite(sample_hz) || sample_hz < 1 || sample_hz > 1000)
            throw std::runtime_error("monitoring.sample_hz must be in [1, 1000]");

        ShmStatistics statistics(shm_name, monitoring["window_sec"].as<double>());
        fs::path log_directory;

        if (!no_terminals) {
            const auto base =
                fs::absolute(config_path.parent_path() / config["log_directory"].as<std::string>());
            fs::create_directories(base);
            const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                                       .count();
            log_directory =
                base / ("run-" + std::to_string(timestamp) + "-" + std::to_string(::getpid()));

            if (!fs::create_directory(log_directory))
                throw std::runtime_error(
                    "process log directory already exists: " + log_directory.string());
        }

        shared_memory = std::make_unique<shm_utils::Mapping>(shm_name, true);
        shared_memory->get()->can_enabled = !without_can;
        statistics.Sample(*shared_memory->get());
        daemon.DisplayStatistics(statistics.Report(shm_utils::MonotonicNs()));
        std::cout << "[daemon] SHM created: " << shm_name << " (" << kShmByteSize << " bytes)"
                  << std::endl;

        if (!no_terminals)
            std::cout << "[daemon] process logs: " << log_directory.string() << std::endl;

        if (without_can)
            std::cout << "[daemon] DEBUG --without-can: using debug_processes without CAN HAL"
                      << std::endl;

        if (no_terminals)
            std::cout << "[daemon] explicit --no-terminals: child output uses this stream"
                      << std::endl;

        for (const auto& spec : specs) {
            if (spec.terminal_title.empty()) {
                StartProcess(daemon, spec, children);
            } else {
                auto terminal = std::make_unique<ProcessTerminal>();
                terminal->Open();
                const auto log_path = log_directory / (spec.name + ".log");
                const int log_fd =
                    ::open(log_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);

                if (log_fd < 0)
                    throw std::runtime_error("cannot create process log: " + log_path.string());

                ::close(log_fd);
                auto application = spec;
                application.output_fd = terminal->slave;
                application.env["QHRR_TERMINAL_STATISTICS"] = "1";

                try {
                    StartProcess(daemon, application, children);
                } catch (...) {
                    terminal->ReportStartupError(log_path);
                    throw;
                }

                ::close(terminal->slave);
                terminal->slave = -1;

                ProcessSpec display;
                display.name = spec.name + ".terminal";
                display.terminal_viewer = true;
                display.exec_path = spec.terminal_exec;
                display.args = {"--wait",
                    "--window",
                    "--geometry=140x34",
                    "--title=" + spec.terminal_title,
                    "--fd=" + std::to_string(terminal->master),
                    "--",
                    (ExecutableDirectory() / "daemon").string(),
                    "--terminal-output",
                    std::to_string(terminal->master),
                    std::to_string(daemon.Get(spec.name)->pid),
                    spec.name,
                    std::to_string(display_interval),
                    log_path.string()};
                display.pass_fd = terminal->master;
                StartProcess(daemon, display, children);
                terminals.push_back(std::move(terminal));
            }

            if (stop_requested)
                break;
        }

        int result = 0;
        bool completed = false;
        auto next_sample = shm_utils::MonotonicNs();
        auto next_display = next_sample + static_cast<std::uint64_t>(display_interval * 1e9);
        const auto sample_period_ns = static_cast<std::uint64_t>(1e9 / sample_hz);

        while (!stop_requested && !completed) {
            for (const auto& name : daemon.Reap()) {
                const auto info = daemon.Get(name).value();
                completed = true;

                if (info.exit_code || info.term_signal)
                    result = info.exit_code ? info.exit_code : 128 + info.term_signal;
            }

            const auto now = shm_utils::MonotonicNs();

            if (now >= next_sample) {
                statistics.Sample(*shared_memory->get());
                next_sample = now + sample_period_ns;
            }

            if (now >= next_display) {
                daemon.DisplayStatistics(statistics.Report(shm_utils::MonotonicNs()));
                next_display = now + static_cast<std::uint64_t>(display_interval * 1e9);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (StopChildren(daemon, children) && result == 0)
            result = 1;

        daemon.ResetStatistics();

        for (const auto& spec : children) {
            const auto info = daemon.Get(spec.name);

            if (info)
                std::cout << "[daemon] " << spec.name << " exited code=" << info->exit_code
                          << " signal=" << info->term_signal << std::endl;

            if (info && result == 0 && info->exit_code != 0)
                result = info->exit_code;

            if (info && result == 0 && info->term_signal != 0 && info->term_signal != SIGTERM)
                result = 128 + info->term_signal;
        }

        return result;
    } catch (const std::exception& error) {
        StopChildren(daemon, children);
        daemon.ResetStatistics();
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';

        return 1;
    }
}
