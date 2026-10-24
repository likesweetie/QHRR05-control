#include "daemon/daemon.hpp"

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <map>

namespace {
namespace fs = std::filesystem;
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }

void CheckKeys(const YAML::Node& node, std::initializer_list<const char*> keys) {
    if (!node || !node.IsMap()) throw std::runtime_error("configuration section must be a map");
    for (const char* key : keys)
        if (!node[key]) throw std::runtime_error(std::string("missing daemon key: ") + key);

}
fs::path Resolve(const fs::path& root, const std::string& value) {
    const auto path = fs::absolute(root / value).lexically_normal();
    if (!fs::is_regular_file(path)) throw std::runtime_error("required file not found: " + path.string());
    return path;
}
fs::path ExecutableDirectory() {
    char buffer[4096];
    const auto size = ::readlink("/proc/self/exe", buffer, sizeof(buffer));
    if (size <= 0 || size == sizeof(buffer)) throw std::runtime_error("cannot resolve daemon executable directory");
    return fs::path(std::string(buffer, size)).parent_path();
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    Daemon daemon;
    try {
        fs::path config_path = QHRR05_DEFAULT_DAEMON_CONFIG;
        std::string control_override;
        std::vector<std::string> forwarded;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") {
                std::cout << "daemon [--config YAML] [--control-config YAML] [--robot qhrr-05] "
                             "[--headless --duration SEC] [--csv PATH]\n";
                return 0;
            }
            if (argument == "--headless") { forwarded.push_back(argument); continue; }
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + argument);
            const std::string value = argv[++i];
            if (argument == "--config") config_path = value;
            else if (argument == "--control-config") control_override = value;
            else if (argument == "--robot") {
                if (value != "qhrr-05") throw std::runtime_error("unsupported robot: " + value);
            } else if (argument == "--duration" || argument == "--csv") {
                forwarded.push_back(argument); forwarded.push_back(value);
            } else throw std::runtime_error("unknown argument: " + argument);
        }
        config_path = fs::absolute(config_path);
        const auto config = YAML::LoadFile(config_path.string());
        CheckKeys(config, {"robots", "processes"});
        const auto robots = config["robots"];
        if (!robots.IsMap() || robots.size() != 1 || !robots["qhrr-05"])
            throw std::runtime_error("robots must contain only qhrr-05");
        const auto selected = robots["qhrr-05"];
        CheckKeys(selected, {"model_path", "control_config_path"});
        const auto root = config_path.parent_path();
        const auto model = Resolve(root, selected["model_path"].as<std::string>());
        const auto control = control_override.empty()
            ? Resolve(root, selected["control_config_path"].as<std::string>())
            : Resolve(fs::current_path(), control_override);
        const auto processes = config["processes"];
        if (!processes.IsSequence() || processes.size() != 2)
            throw std::runtime_error("task_controller and mujoco_simulate processes are required");
        std::map<std::string, std::string> executables;
        for (const auto& process : processes) {
            CheckKeys(process, {"name", "exec_path"});
            const auto name = process["name"].as<std::string>();
            if ((name != "task_controller" && name != "mujoco_simulate") || executables.count(name))
                throw std::runtime_error("only one task_controller and one mujoco_simulate are supported");
            executables[name] = Resolve(ExecutableDirectory(), process["exec_path"].as<std::string>()).string();
        }
        int channel[2];
        if (::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, channel) != 0)
            throw std::runtime_error(std::string("control socketpair failed: ") + std::strerror(errno));
        try {
            for (const auto& name : {"task_controller", "mujoco_simulate"}) {
                const bool controller = std::string(name) == "task_controller";
                const int fd = channel[controller ? 0 : 1];
                ProcessSpec spec;
                spec.name = name;
                spec.exec_path = executables.at(name);
                spec.env["ROBOT_NAME"] = "qhrr-05";
                spec.inherited_fds = {fd};
                if (controller) spec.args = {control.string(), std::to_string(fd)};
                else {
                    spec.args = {"--model", model.string(), "--control-config", control.string(), "--io-fd", std::to_string(fd)};
                    spec.args.insert(spec.args.end(), forwarded.begin(), forwarded.end());
                }
                const auto pid = daemon.Spawn(spec);
                if (!pid) throw std::runtime_error(std::string("exec failed: ") + std::strerror(daemon.LastErrno()));
                std::cout << "[daemon] " << name << " started pid=" << *pid << std::endl;
            }
        } catch (...) {
            ::close(channel[0]); ::close(channel[1]);
            daemon.TerminateAll(SIGTERM);
            for (const auto& [name, _] : executables) {
                while (daemon.Get(name) && daemon.Get(name)->running) {
                    daemon.Reap();
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }
            throw;
        }
        ::close(channel[0]); ::close(channel[1]);
        int result = 0;
        while (!stop_requested) {
            const auto exited = daemon.Reap();
            if (!exited.empty()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // Let a normally closed viewer send its stop frame before forced shutdown.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        if (stop_requested) daemon.TerminateAll(SIGTERM);
        for (;;) {
            daemon.Reap();
            bool running = false;
            for (const auto& [name, _] : executables) {
                const auto info = daemon.Get(name).value();
                running |= info.running;
                if (info.exited && (info.exit_code != 0 || (info.term_signal != 0 && info.term_signal != SIGTERM))) {
                    result = info.exit_code != 0 ? info.exit_code : 128+info.term_signal;
                    daemon.TerminateAll(SIGTERM);
                }
            }
            if (!running) break;
            if (std::chrono::steady_clock::now() >= deadline) {
                std::cerr << "[daemon] stopping remaining owned processes\n";
                daemon.TerminateAll(SIGKILL);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for (const auto& [name, _] : executables) {
            const auto info = daemon.Get(name).value();
            std::cout << "[daemon] " << name << " exited code=" << info.exit_code
                      << " signal=" << info.term_signal << std::endl;
        }
        return result;
    } catch (const std::exception& error) {
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';
        return 1;
    }
}
