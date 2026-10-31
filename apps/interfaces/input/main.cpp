#include "interfaces/input/joystick.hpp"
#include "interfaces/input/client.hpp"
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
        if (argc >= 4 && std::string(argv[1]) == "--command") {
            qhrr05::InputClient client(argv[2]);
            std::string command;
            for (int i = 3; i < argc; ++i) {
                if (i > 3)
                    command += ' ';
                command += argv[i];
            }
            client.Submit(command);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < deadline) {
                for (const auto& decision : client.Decisions()) {
                    std::cout << (decision.approved ? "APPROVED: " : "REJECTED: ")
                              << decision.message << '\n';
                    return decision.approved ? 0 : 1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            throw std::runtime_error("Input decision timeout; command was not retried");
        }
        if (argc != 3)
            throw std::runtime_error(
                "usage: input SHM_NAME INPUT_CONFIG | input --command SHM_NAME OPERATION [ARGS]");

        shm_utils::Mapping mapping(argv[1]);
        auto& shm = *mapping.get();
        Joystick joystick(argv[2], &shm);
        joystick.initialize();
        joystick.start();
        bool finished = false;

        while (!stopped && joystick.isRunning()) {
            {
                shm_utils::Lock lock(shm);
                finished = shm.stopping;
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
