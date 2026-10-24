#include "daemon/daemon.hpp"
#include "homing_controller.hpp"

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }

void CheckKeys(const YAML::Node& node, std::initializer_list<const char*> keys) {
    if (!node || !node.IsMap()) throw std::runtime_error("configuration section must be a map");
    for (const char* key : keys)
        if (!node[key]) throw std::runtime_error(std::string("missing daemon key: ") + key);
    for (const auto& entry : node) {
        const std::string key = entry.first.as<std::string>();
        bool known = false;
        for (const char* expected : keys) known |= key == expected;
        if (!known) throw std::runtime_error("unknown daemon key: " + key);
    }
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
                if (value != qhrr05::kRobotName) throw std::runtime_error("unsupported robot: " + value);
            } else if (argument == "--duration" || argument == "--csv") {
                forwarded.push_back(argument); forwarded.push_back(value);
            } else throw std::runtime_error("unknown argument: " + argument);
        }
        config_path = fs::absolute(config_path);
        const auto config = YAML::LoadFile(config_path.string());
        CheckKeys(config, {"robots", "processes"});
        const auto robots = config["robots"];
        if (!robots.IsMap() || robots.size() != 1 || !robots[qhrr05::kRobotName])
            throw std::runtime_error("robots must contain only qhrr-05");
        const auto selected = robots[qhrr05::kRobotName];
        CheckKeys(selected, {"model_path", "control_config_path"});
        const auto root = config_path.parent_path();
        const auto model = Resolve(root, selected["model_path"].as<std::string>());
        const auto control = control_override.empty()
            ? Resolve(root, selected["control_config_path"].as<std::string>())
            : Resolve(fs::current_path(), control_override);
        (void)qhrr05::LoadControlConfig(control.string());
        const auto processes = config["processes"];
        if (!processes.IsSequence() || processes.size() != 1)
            throw std::runtime_error("exactly one task_controller process is required");
        const auto process = processes[0];
        CheckKeys(process, {"name", "exec_path"});
        if (process["name"].as<std::string>() != "task_controller")
            throw std::runtime_error("only the task_controller process is supported");
        ProcessSpec spec;
        spec.name = "task_controller";
        spec.exec_path = Resolve(ExecutableDirectory(), process["exec_path"].as<std::string>()).string();
        spec.env["ROBOT_NAME"] = qhrr05::kRobotName;
        spec.args = {"--model", model.string(), "--control-config", control.string()};
        spec.args.insert(spec.args.end(), forwarded.begin(), forwarded.end());
        const auto pid = daemon.Spawn(spec);
        if (!pid) throw std::runtime_error(std::string("exec failed: ") + std::strerror(daemon.LastErrno()));
        std::cout << "[daemon] qhrr-05 homing started pid=" << *pid << " config=" << control << std::endl;
        int result = 0;
        while (!stop_requested) {
            const auto exited = daemon.Reap();
            if (!exited.empty()) {
                const auto status = daemon.Get(spec.name).value();
                result = status.term_signal ? 128+status.term_signal : status.exit_code;
                std::cout << "[daemon] task_controller exited code=" << result << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        daemon.TerminateAll(SIGTERM);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (daemon.Get(spec.name)->running && std::chrono::steady_clock::now() < deadline) {
            daemon.Reap();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (daemon.Get(spec.name)->running) {
            std::cerr << "[daemon] owned child did not stop; sending SIGKILL\n";
            daemon.TerminateAll(SIGKILL);
            while (daemon.Get(spec.name)->running) {
                daemon.Reap();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        return result;
    } catch (const std::exception& error) {
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';
        return 1;
    }
}
