#include "task_controller.hpp"
#include <iostream>
#include <string>
#include <csignal>

namespace {
volatile std::sig_atomic_t stopping = 0;

void OnSignal(int) {
    stopping = 1;
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try {
        const char* usage = "usage: task_controller CONTROL_CONFIG SHM_NAME "
                            "[--without-can]";

        if (argc < 3)
            throw std::runtime_error(usage);

        bool without_can = false;

        for (int i = 3; i < argc; ++i) {
            const std::string argument = argv[i];

            if (argument == "--without-can")
                without_can = true;
            else
                throw std::runtime_error(usage);
        }

        qhrr05::TaskController task(argv[1], argv[2], without_can);

        while (!stopping && task.ReadInput()) {
            task.Control();
            task.WriteOutput();
            task.WaitNextCycle();
        }

        task.Release();

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[task_controller] " << error.what() << '\n';
        return 1;
    }
}
