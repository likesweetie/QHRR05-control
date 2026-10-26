#include "input/joystick.hpp"
#include "ipc/shm_utils.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;

void OnSignal(int) {
    stopped = 1;
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try {
        if (argc != 3)
            throw std::runtime_error("usage: input SHM_NAME JOYSTICK_DEVICE");

        shm_utils::Mapping mapping(argv[1]);
        auto& shm = *mapping.get();
        Joystick joystick(argv[2], &shm);
        joystick.initialize();
        joystick.start();
        bool finished = false;

        while (!stopped && joystick.isRunning()) {
            {
                shm_utils::Lock lock(shm);
                finished = shm.stopping || shm.simulation_finished;
            }

            if (finished)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        const bool device_failed = !stopped && !finished && !joystick.isRunning();
        joystick.stop();
        joystick.join();

        if (device_failed)
            throw std::runtime_error("joystick reader stopped unexpectedly");

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[input] " << error.what() << '\n';
        return 1;
    }
}
