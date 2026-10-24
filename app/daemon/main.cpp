#include "daemon/daemon.hpp"
#include "shm_utils.hpp"
#include <memory>

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <map>

namespace {
namespace fs = std::filesystem;
volatile std::sig_atomic_t stop_requested = 0;

void OnSignal(int) {
    stop_requested = 1;
}

void CheckKeys(const YAML::Node& node, std::initializer_list<const char*> keys) {
    if (!node || !node.IsMap())
        throw std::runtime_error("configuration section must be a map");
    for (const char* key : keys)
        if (!node[key])
            throw std::runtime_error(std::string("missing daemon key: ") + key);
}

fs::path Resolve(const fs::path& root, const std::string& value) {
    const auto path = fs::absolute(root / value).lexically_normal();
    if (!fs::is_regular_file(path))
        throw std::runtime_error("required file not found: " + path.string());
    return path;
}

fs::path ExecutableDirectory() {
    char buffer[4096];
    const auto size = ::readlink("/proc/self/exe", buffer, sizeof(buffer));
    if (size <= 0 || size == sizeof(buffer))
        throw std::runtime_error("cannot resolve daemon executable directory");
    return fs::path(std::string(buffer, size)).parent_path();
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::unique_ptr<shm_utils::Mapping> shm;
    Daemon daemon;
    try {
        fs::path config_path = QHRR05_DEFAULT_DAEMON_CONFIG;
        std::string control_override, shm_override, csv_path;
        std::vector<std::string> forwarded;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") {
                std::cout << "daemon [--config YAML] [--control-config YAML] [--robot qhrr-05] "
                             "[--shm-name NAME] [--headless --duration SEC] [--csv PATH]\n";
                return 0;
            }
            if (argument == "--headless") {
                forwarded.push_back(argument);
                continue;
            }
            if (i + 1 >= argc)
                throw std::runtime_error("missing value for " + argument);
            const std::string value = argv[++i];
            if (argument == "--config")
                config_path = value;
            else if (argument == "--control-config")
                control_override = value;
            else if (argument == "--shm-name")
                shm_override = value;
            else if (argument == "--csv")
                csv_path = value;
            else if (argument == "--robot") {
                if (value != "qhrr-05")
                    throw std::runtime_error("unsupported robot: " + value);
            } else if (argument == "--duration") {
                forwarded.push_back(argument);
                forwarded.push_back(value);
            } else
                throw std::runtime_error("unknown argument: " + argument);
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
                throw std::runtime_error(
                    "only one task_controller and one mujoco_simulate are supported");
            executables[name] =
                Resolve(ExecutableDirectory(), process["exec_path"].as<std::string>()).string();
        }
        if (!csv_path.empty())
            executables["shm_logger"] = Resolve(ExecutableDirectory(), "shm_logger").string();
        const auto shm_name =
            shm_override.empty() ? config["shm_name"].as<std::string>() : shm_override;
        shm = std::make_unique<shm_utils::Mapping>(shm_name, true);
        std::cout << "[daemon] SHM " << shm_name << " (sequence double buffers)" << std::endl;
        // Start readers before the simulator. The daemon owns creation and unlink.
        for (const auto& name : {"task_controller", "shm_logger", "mujoco_simulate"}) {
            if (!executables.count(name))
                continue;
            ProcessSpec spec;
            spec.name = name;
            spec.exec_path = executables.at(name);
            if (spec.name == "task_controller")
                spec.args = {control.string(), shm_name};
            else if (spec.name == "shm_logger")
                spec.args = {shm_name, csv_path};
            else {
                spec.args = {"--model",
                    model.string(),
                    "--control-config",
                    control.string(),
                    "--shm-name",
                    shm_name};
                spec.args.insert(spec.args.end(), forwarded.begin(), forwarded.end());
            }
            const auto pid = daemon.Spawn(spec);
            if (!pid)
                throw std::runtime_error(
                    std::string("exec failed: ") + std::strerror(daemon.LastErrno()));
            std::cout << "[daemon] " << name << " started pid=" << *pid << std::endl;
        }
        int result = 0;
        while (!stop_requested) {
            const auto exited = daemon.Reap();
            if (!exited.empty()) {
                bool finished;
                {
                    shm_utils::Lock lock(*shm->get());
                    finished = shm->get()->simulation_finished;
                }
                if (finished)
                    shm_utils::Stop(*shm->get());
                if (!finished) {
                    std::cerr << "[daemon] child exited before simulation finished\n";
                    result = 1;
                    shm_utils::Stop(*shm->get(), true);
                    daemon.TerminateAll(SIGTERM);
                }
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // A finished simulator wakes the controller and logger through SHM.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        if (stop_requested) {
            shm_utils::Stop(*shm->get());
            daemon.TerminateAll(SIGTERM);
        }
        for (;;) {
            daemon.Reap();
            bool running = false;
            for (const auto& [name, _] : executables) {
                const auto info = daemon.Get(name).value();
                running |= info.running;
                if (info.exited && (info.exit_code != 0 ||
                                       (info.term_signal != 0 && info.term_signal != SIGTERM))) {
                    result = info.exit_code != 0 ? info.exit_code : 128 + info.term_signal;
                    shm_utils::Stop(*shm->get(), true);
                    daemon.TerminateAll(SIGTERM);
                }
            }
            if (!running)
                break;
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
        if (shm) {
            try {
                shm_utils::Stop(*shm->get(), true);
            } catch (const std::exception& stop_error) {
                std::cerr << "[daemon] " << stop_error.what() << '\n';
            }
        }
        daemon.TerminateAll(SIGTERM);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            daemon.Reap();
            bool running = false;
            for (const char* name : {"task_controller", "shm_logger", "mujoco_simulate"}) {
                const auto child = daemon.Get(name);
                running |= child && child->running;
            }
            if (!running)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        daemon.TerminateAll(SIGKILL);
        daemon.Reap();
        std::cerr << "[daemon] ERROR: " << error.what() << '\n';
        return 1;
    }
}
