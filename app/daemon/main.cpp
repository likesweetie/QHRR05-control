#include "daemon/daemon.hpp"

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
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

std::vector<ProcessSpec> LoadProcesses(const fs::path& path, bool without_can) {
    const auto config = YAML::LoadFile(path.string());
    const std::string section = without_can ? "debug_processes" : "processes";
    const auto processes = config[section];

    if (!config.IsMap() || !processes || !processes.IsSequence() || processes.size() == 0)
        throw std::runtime_error("daemon config requires a nonempty " + section + " sequence");

    std::vector<ProcessSpec> result;

    for (const auto& process : processes) {
        ProcessSpec spec;
        spec.name = process["name"].as<std::string>();
        spec.exec_path =
            fs::absolute(ExecutableDirectory() / process["exec_path"].as<std::string>()).string();
        spec.args = process["args"].as<std::vector<std::string>>();

        if (spec.name.empty() || !fs::is_regular_file(spec.exec_path))
            throw std::runtime_error("invalid process name or executable: " + spec.name);

        for (const auto& existing : result)
            if (existing.name == spec.name)
                throw std::runtime_error("duplicate process name: " + spec.name);

        if (process["working_directory"])
            spec.working_directory =
                fs::absolute(path.parent_path() / process["working_directory"].as<std::string>())
                    .string();

        if (process["wait_ready"])
            spec.wait_ready = process["wait_ready"].as<bool>();

        if (process["terminal_title"]) {
            spec.terminal_title = process["terminal_title"].as<std::string>();
            spec.terminal_exec =
                fs::absolute(ExecutableDirectory() / process["terminal_exec"].as<std::string>())
                    .string();

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

    void ReportStartupError() {
        if (::fcntl(master, F_SETFL, O_NONBLOCK) < 0)
            throw std::runtime_error("cannot read process startup output");

        char buffer[4096];

        while (true) {
            const auto count = ::read(master, buffer, sizeof(buffer));

            if (count > 0)
                std::cerr.write(buffer, count);
            else if (count < 0 && errno == EINTR)
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
    daemon.TerminateAll(SIGTERM);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (AnyRunning(daemon, specs) && std::chrono::steady_clock::now() < deadline) {
        daemon.Reap();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const bool forced = AnyRunning(daemon, specs);
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
    if (argc == 3 && std::string(argv[1]) == "--terminal-output")
        return Daemon::RelayTerminalOutput(argv[2]);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGHUP, OnSignal);
    Daemon daemon;
    std::vector<ProcessSpec> children;
    std::vector<std::unique_ptr<ProcessTerminal>> terminals;

    try {
        fs::path config_path = QHRR05_DEFAULT_DAEMON_CONFIG;
        bool without_can = false;
        const char* usage = "usage: daemon [--config PROCESS_CONFIG.yaml] [--without-can]";

        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];

            if (argument == "--without-can")
                without_can = true;
            else if (argument == "--config" && i + 1 < argc)
                config_path = argv[++i];
            else if (argument == "--help") {
                std::cout << usage << '\n';
                return 0;
            } else
                throw std::runtime_error(usage);
        }

        const auto specs = LoadProcesses(fs::absolute(config_path), without_can);

        if (without_can)
            std::cout << "[daemon] DEBUG --without-can: using debug_processes without CAN HAL"
                      << std::endl;

        for (const auto& spec : specs) {
            if (spec.terminal_title.empty()) {
                StartProcess(daemon, spec, children);
            } else {
                auto terminal = std::make_unique<ProcessTerminal>();
                terminal->Open();
                auto application = spec;
                application.output_fd = terminal->slave;

                try {
                    StartProcess(daemon, application, children);
                } catch (...) {
                    terminal->ReportStartupError();
                    throw;
                }

                ::close(terminal->slave);
                terminal->slave = -1;

                ProcessSpec display;
                display.name = spec.name + ".terminal";
                display.exec_path = spec.terminal_exec;
                display.args = {"--wait",
                    "--window",
                    "--title=" + spec.terminal_title,
                    "--fd=" + std::to_string(terminal->master),
                    "--",
                    (ExecutableDirectory() / "daemon").string(),
                    "--terminal-output",
                    std::to_string(terminal->master)};
                display.pass_fd = terminal->master;
                StartProcess(daemon, display, children);
                terminals.push_back(std::move(terminal));
            }

            if (stop_requested)
                break;
        }

        int result = 0;
        bool completed = false;

        while (!stop_requested && !completed) {
            for (const auto& name : daemon.Reap()) {
                const auto info = daemon.Get(name).value();
                completed = true;

                if (info.exit_code || info.term_signal)
                    result = info.exit_code ? info.exit_code : 128 + info.term_signal;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (StopChildren(daemon, children) && result == 0)
            result = 1;

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
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';

        return 1;
    }
}
