#include "joystick.hpp"
#include "shm_utils.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_run{true};

void HandleSignal(int) {
    g_run.store(false, std::memory_order_relaxed);
}

const char* ResolveShmName() {
    const char* name = std::getenv("SHM_NAME");
    return (name && name[0]) ? name : "/shm0";
}

const char* ResolveJoystickDev(int argc, char** argv) {
    if (argc > 1 && argv[1] && argv[1][0]) {
        return argv[1];
    }
    const char* env = std::getenv("JOYSTICK_DEV");
    return (env && env[0]) ? env : "/dev/input/js0";
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const char* shm_name = ResolveShmName();
    const char* joystick_dev = ResolveJoystickDev(argc, argv);

    std::cout << "[aux_reader] waiting shm: " << shm_name << std::endl;

    ShmData* shm = nullptr;
    while (g_run.load(std::memory_order_relaxed) && !shm) {
        shm = shm_utils::OpenShm(shm_name);
        if (!shm) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!shm) {
        std::cerr << "[aux_reader] failed to open shm" << std::endl;
        return 1;
    }

    std::cout << "[aux_reader] joystick device: " << joystick_dev << std::endl;
    Joystick joystick(joystick_dev, shm);
    joystick.initialize();
    joystick.start();

    while (g_run.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    joystick.stop();
    joystick.join();
    shm_utils::CloseShm(shm);
    return 0;
}
