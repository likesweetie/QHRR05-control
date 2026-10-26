#include "shm_logger/logger_service.hpp"
#include <csignal>
#include <iostream>

namespace {
volatile std::sig_atomic_t stopping = 0;

void Stop(int) {
    stopping = 1;
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);
    std::signal(SIGHUP, Stop);
    std::signal(SIGPIPE, SIG_IGN);
    try {
        if (argc == 4 && std::string(argv[1]) == "--csv")
            return qhrr05::logging::RunCsvLogger(argc - 1, argv + 1);
        if (argc == 4 && std::string(argv[1]) == "--can")
            return qhrr05::logging::RunCsvLogger(argc, argv);
        std::string config, shm, daemon;
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (i + 1 == argc)
                throw std::runtime_error("missing logger argument value");
            const std::string value = argv[++i];
            if (key == "--config")
                config = value;
            else if (key == "--shm-name")
                shm = value;
            else if (key == "--daemon-config")
                daemon = value;
            else
                throw std::runtime_error("unknown logger argument: " + key);
        }
        if (config.empty() || shm.empty())
            throw std::runtime_error(
                "usage: shm_logger --config LOGGER.yaml --shm-name SHM [--daemon-config DAEMON.yaml]; explicit CSV: --csv SHM FILE or --can SHM FILE");
        qhrr05::logging::LoggerService service(config, shm, daemon);
        return service.Run(stopping);
    } catch (const std::exception& e) {
        std::cerr << "[shm_logger] ERROR: " << e.what() << '\n';
        return 1;
    }
}
