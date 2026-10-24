#include "task_controller.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc != 3)
            throw std::runtime_error("usage: task_controller CONTROL_CONFIG SHM_NAME");

        qhrr05::TaskController task(argv[1], argv[2]);

        while (task.ReadInput()) {
            task.Control();
            task.WriteOutput();
            task.WaitNextCycle();
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[task_controller] " << error.what() << '\n';
        return 1;
    }
}
