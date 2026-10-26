#include "shm_utils.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <sys/wait.h>
#include <vector>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

std::vector<std::string> Fields(const std::string& line) {
    std::vector<std::string> values;
    std::istringstream stream(line);
    std::string value;

    while (std::getline(stream, value, ','))
        values.push_back(value);

    return values;
}

int Run(const char* logger, const std::string& name, const std::string& csv) {
    const pid_t child = ::fork();
    Require(child >= 0, "fork failed");

    if (child == 0) {
        ::execl(logger, logger, name.c_str(), csv.c_str(), static_cast<char*>(nullptr));
        ::_exit(127);
    }

    int status = 0;

    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
    }

    Require(WIFEXITED(status), "logger did not exit normally");
    return WEXITSTATUS(status);
}

int main(int argc, char** argv) {
    char directory[] = "/tmp/qhrr05-logger-XXXXXX";

    try {
        Require(argc == 2, "expected shm_logger executable");
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        const auto name = "/qhrr05-logger-" + std::to_string(::getpid());
        const std::string csv = std::string(directory) + "/data.csv";
        shm_utils::Mapping mapping(name, true);
        auto& shm = *mapping.get();
        {
            shm_utils::Lock lock(shm);
            auto& sample = shm.simulation[1];
            shm.simulation_sequence = 5;  // Reader starts after four missed publications.
            sample.run_id = 7;
            shm.control[0].input_model_sequence = 42;
            shm.control[0].command_sequence = 17;
            shm.control[0].control_time = 1.25;
            shm.control[0].mode = 1;
            sample.time = .123;
            sample.contact = true;
            sample.contact_event_sequence = 2;
            sample.contact_event_time = .08;
            sample.position_gt = {.1, .2, .3};
            const auto now = shm_utils::MonotonicNs();
            shm.control[0].input_model_published_ns = now - 5000000;
            shm.control[0].published_ns = now - 2000000;
            sample.published_ns = now;
            shm.simulation_finished = true;
        }

        Require(Run(argv[1], name, csv) == 0, "logger failed");
        std::ifstream input(csv);
        std::string header, row, extra;
        std::getline(input, header);
        std::getline(input, row);
        std::getline(input, extra);
        const auto keys = Fields(header), values = Fields(row);
        Require(keys.size() == values.size() && extra.empty(), "wrong CSV shape");
        std::map<std::string, std::string> data;

        for (std::size_t i = 0; i < keys.size(); ++i)
            data[keys[i]] = values[i];

        Require(
            data["controller"] == "gravity" && data["run_id"] == "7", "wrong telemetry metadata");
        Require(
            data["state_seq"] == "42" && data["command_seq"] == "17", "sequence association lost");
        Require(
            std::stod(data["control_latency_us"]) == 3000, "logger latency calculation is wrong");
        Require(std::stod(data["sample_age_us"]) >= 0, "logger sample age is wrong");
        Require(std::stod(data["control_time"]) == 1.25,
            "controller time was not recorded independently");
        Require(
            data["skipped_samples"] == "4", "logger did not evaluate missed double-buffer samples");
        Require(data["sim_contact"] == "1" && data["sim_contact_event_seq"] == "2" &&
                    std::stod(data["sim_contact_event_time"]) == .08,
            "simulation contact events were not read from their canonical block");
        Require(Run(argv[1], name, csv) == 1, "logger overwrote an existing CSV");
        std::ifstream unchanged(csv);
        std::string same_header, same_row;
        std::getline(unchanged, same_header);
        std::getline(unchanged, same_row);
        Require(header == same_header && row == same_row, "existing CSV was modified");
        std::filesystem::remove_all(directory);
        std::cout
            << "PASS: logger-only timing/missed-sequence evaluation, existing CSV rejection\n";
        return 0;
    } catch (const std::exception& error) {
        std::filesystem::remove_all(directory);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
