#include "task_controller.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: task_controller CONTROL_CONFIG IO_FD");
        std::size_t parsed = 0;
        const std::string argument = argv[2];
        const int fd = std::stoi(argument, &parsed);
        if (parsed != argument.size()) throw std::runtime_error("invalid IO_FD");
        qhrr05::TaskController task(YAML::LoadFile(argv[1]), fd);
        while (task.ReadInput()) {
            task.Control();
            task.WriteOutput();
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[task_controller] " << error.what() << '\n';
        return 1;
    }
}
