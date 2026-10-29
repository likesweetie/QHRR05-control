#include "daemon/daemon.hpp"
#include "ipc/shm_utils.hpp"
#include "daemon/statistics.hpp"

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
#include <algorithm>

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
        if (process["critical"])
            spec.critical = process["critical"].as<bool>();
        spec.exec_path =
            fs::absolute(ExecutableDirectory() / process["exec_path"].as<std::string>()).string();
        spec.args = process["args"].as<std::vector<std::string>>();

        for (auto& argument : spec.args)
            if (argument == "{shm_name}")
                argument = shm_name;
            else if (argument == "{daemon_config}")
                argument = fs::absolute(path).string();

        if (spec.name.empty() || (spec.critical && !fs::is_regular_file(spec.exec_path)))
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

            if (spec.terminal_title.empty() ||
                (spec.critical && !fs::is_regular_file(spec.terminal_exec)))
                throw std::runtime_error("invalid terminal configuration: " + spec.name);
        }

        if (process["ready_timeout_ms"])
            spec.ready_timeout_ms = process["ready_timeout_ms"].as<int>();

        if (spec.ready_timeout_ms <= 0)
            throw std::runtime_error("ready_timeout_ms must be positive");

        if (process["critical"])
            spec.critical = process["critical"].as<bool>();
        if (process["restart_limit"])
            spec.restart_limit = process["restart_limit"].as<int>();
        if (process["restart_delay_ms"])
            spec.restart_delay_ms = process["restart_delay_ms"].as<int>();
        if (spec.restart_limit < 0 || spec.restart_limit > 20 || spec.restart_delay_ms < 100 ||
            spec.restart_delay_ms > 60000)
            throw std::runtime_error("invalid process restart policy: " + spec.name);
        if (spec.critical && spec.restart_limit)
            throw std::runtime_error("only noncritical processes may restart: " + spec.name);

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

    const auto previous =
        std::find_if(children.begin(), children.end(), [&](const ProcessSpec& old) {
            return old.name == spec.name;
        });
    if (previous == children.end())
        children.push_back(spec);
    else
        *previous = spec;
    daemon.WriteTerminalLog(
        "[daemon] " + spec.name + " started pid=" + std::to_string(*pid) + "\n");
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
    // Critical terminals stop the daemon; optional terminals stop only their application.
    if (argc == 8 && std::string(argv[1]) == "--terminal-output")
        return Daemon::RelayTerminalOutput(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);

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

        daemon.BeginTerminalScreen();
        config_path = fs::absolute(config_path);
        const auto config = YAML::LoadFile(config_path.string());
        auto specs = LoadProcesses(config_path, config, without_can, no_terminals);
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
            const auto layout = config["terminal_layout"];
            daemon.PlanTerminalLayout(specs,
                layout["monitor"].as<std::string>(),
                layout["columns"].as<int>(),
                layout["gap_px"].as<int>(),
                layout["margin_px"].as<int>(),
                layout["zoom"].as<double>(),
                layout["include_daemon"].as<bool>());

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
        daemon.DisplayStatistics(
            statistics.Report(shm_utils::MonotonicNs(), ::isatty(STDOUT_FILENO)));
        daemon.WriteTerminalLog("[daemon] SHM created: " + shm_name + " (" +
                                std::to_string(kShmByteSize) + " bytes)\n");

        if (!no_terminals)
            daemon.WriteTerminalLog("[daemon] process logs: " + log_directory.string() + "\n");

        if (without_can)
            daemon.WriteTerminalLog(
                "[daemon] DEBUG --without-can: using debug_processes without CAN HAL\n");

        if (no_terminals)
            daemon.WriteTerminalLog(
                "[daemon] explicit --no-terminals: child output uses this stream\n");

        std::map<std::string, int> restart_count;
        std::map<std::string, std::uint64_t> restart_at;
        const auto launch = [&](const ProcessSpec& spec) {
            if (spec.terminal_title.empty()) {
                StartProcess(daemon, spec, children);
            } else {
                auto terminal = std::make_unique<ProcessTerminal>();
                terminal->Open();
                const auto suffix = restart_count[spec.name]
                                        ? ".restart-" + std::to_string(restart_count[spec.name])
                                        : std::string{};
                const auto log_path = log_directory / (spec.name + suffix + ".log");
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
                } catch (const std::exception& error) {
                    terminal->ReportStartupError(log_path);
                    throw std::runtime_error(
                        std::string(error.what()) + "; startup log: " + log_path.string());
                }

                ::close(terminal->slave);
                terminal->slave = -1;

                ProcessSpec display;
                display.name = spec.name + ".terminal";
                display.terminal_viewer = true;
                display.critical = spec.critical;
                display.exec_path = spec.terminal_exec;
                display.args = {"--wait",
                    "--window",
                    "--geometry=100x28+" + std::to_string(spec.terminal_x) + "+" +
                        std::to_string(spec.terminal_y),
                    "--zoom=" + std::to_string(spec.terminal_zoom),
                    "--role=" + spec.terminal_role,
                    "--title=" + spec.terminal_title,
                    "--fd=" + std::to_string(terminal->master),
                    "--",
                    (ExecutableDirectory() / "daemon").string(),
                    "--terminal-output",
                    std::to_string(terminal->master),
                    std::to_string(daemon.Get(spec.name)->pid),
                    spec.name,
                    std::to_string(display_interval),
                    log_path.string(),
                    std::to_string(spec.critical ? ::getpid() : daemon.Get(spec.name)->pid)};
                display.pass_fd = terminal->master;
                StartProcess(daemon, display, children);
                daemon.PlaceTerminalWindow(spec);
                terminals.push_back(std::move(terminal));
            }
        };

        for (const auto& spec : specs) {
            try {
                launch(spec);
            } catch (const std::exception& e) {
                if (spec.critical)
                    throw;
                daemon.WriteTerminalLog("[daemon] optional process startup ERROR: " + spec.name +
                                        ": " + e.what() + "\n");
                daemon.Signal(spec.name, SIGTERM);
                restart_at[spec.name] =
                    shm_utils::MonotonicNs() + std::uint64_t(spec.restart_delay_ms) * 1000000;
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
                if (name == "input") {
                    // Process-exit cleanup also covers SIGKILL; no input-age watchdog.
                    auto& shm = *shared_memory->get();
                    shm_utils::Lock lock(shm);
                    auto& input = shm_utils::Next(shm.control, shm.control_sequence);
                    input.joystick_connected = false;
                    input.joystick_axes.fill(0);
                    input.joystick_buttons.fill(false);
                    input.joystick_l2 = input.joystick_r2 = 0;
                    ++input.joystick_sequence;
                    input.joystick_published_ns = shm_utils::MonotonicNs();
                    ++shm.control_sequence;
                    shm_utils::Notify(shm);
                }

                const auto base = name.size() > 9 && name.substr(name.size() - 9) == ".terminal"
                                      ? name.substr(0, name.size() - 9)
                                      : name;
                const auto spec =
                    std::find_if(specs.begin(), specs.end(), [&](const ProcessSpec& item) {
                        return item.name == base;
                    });
                if (spec != specs.end() && !spec->critical) {
                    daemon.WriteTerminalLog("[daemon] optional process exited: " + name +
                                            " code=" + std::to_string(info.exit_code) +
                                            " signal=" + std::to_string(info.term_signal) + "\n");
                    if (name != base)
                        daemon.Signal(base, SIGTERM);
                    if (!restart_at.count(base))
                        restart_at[base] = shm_utils::MonotonicNs() +
                                           std::uint64_t(spec->restart_delay_ms) * 1000000;
                    continue;
                }
                completed = true;
                if (info.exit_code || info.term_signal)
                    result = info.exit_code ? info.exit_code : 128 + info.term_signal;
            }

            const auto now = shm_utils::MonotonicNs();
            if (!completed && !stop_requested) {
                for (const auto& spec : specs) {
                    if (spec.critical || !restart_at.count(spec.name) ||
                        now < restart_at[spec.name])
                        continue;
                    const auto app = daemon.Get(spec.name),
                               viewer = daemon.Get(spec.name + ".terminal");
                    if ((app && app->running) || (viewer && viewer->running))
                        continue;
                    restart_at.erase(spec.name);
                    if (restart_count[spec.name] >= spec.restart_limit) {
                        daemon.WriteTerminalLog(
                            "[daemon] optional restart limit reached: " + spec.name + "\n");
                        continue;
                    }
                    ++restart_count[spec.name];
                    daemon.WriteTerminalLog(
                        "[daemon] restarting optional process: " + spec.name + "\n");
                    try {
                        launch(spec);
                    } catch (const std::exception& e) {
                        daemon.WriteTerminalLog("[daemon] optional restart ERROR: " + spec.name +
                                                ": " + e.what() + "\n");
                        daemon.Signal(spec.name, SIGTERM);
                        restart_at[spec.name] =
                            now + std::uint64_t(spec.restart_delay_ms) * 1000000;
                    }
                }
            }

            if (now >= next_sample) {
                statistics.Sample(*shared_memory->get());
                next_sample = now + sample_period_ns;
            }

            if (now >= next_display) {
                daemon.DisplayStatistics(
                    statistics.Report(shm_utils::MonotonicNs(), ::isatty(STDOUT_FILENO)));
                next_display = now + static_cast<std::uint64_t>(display_interval * 1e9);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (StopChildren(daemon, children) && result == 0)
            result = 1;

        for (const auto& spec : children) {
            const auto info = daemon.Get(spec.name);

            if (info)
                daemon.WriteTerminalLog("[daemon] " + spec.name +
                                        " exited code=" + std::to_string(info->exit_code) +
                                        " signal=" + std::to_string(info->term_signal) + "\n");

            if (spec.critical && info && result == 0 && info->exit_code != 0)
                result = info->exit_code;

            if (spec.critical && info && result == 0 && info->term_signal != 0 &&
                info->term_signal != SIGTERM)
                result = 128 + info->term_signal;
        }

        daemon.ResetStatistics();

        if (result && ::isatty(STDOUT_FILENO))
            std::cerr << "[daemon] stopped with code " << result << "; logs: " << log_directory
                      << '\n';

        return result;
    } catch (const std::exception& error) {
        StopChildren(daemon, children);
        daemon.ResetStatistics();
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';

        return 1;
    }
}
