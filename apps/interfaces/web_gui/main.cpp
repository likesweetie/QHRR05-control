#include "interfaces/web_gui/bridge.hpp"

#include <csignal>
#include <iostream>

int main(int argc, char** argv) {
    std::signal(SIGPIPE, SIG_IGN);

    try {
        if (argc != 3)
            throw std::runtime_error("usage: web_gui_bridge SHM_NAME CONTROL_CONFIG.yaml");

        qhrr05::WebGuiBridge bridge(argv[1], argv[2]);
        bridge.Run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[web_gui_bridge] ERROR: " << error.what() << '\n';
        return 1;
    }
}
