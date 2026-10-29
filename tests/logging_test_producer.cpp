#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

volatile std::sig_atomic_t stopped = 0;

void Stop(int) {
    stopped = 1;
}

int main(int argc, char** argv) {
    try {
        const bool attach = argc > 1 && std::string(argv[1]) == "--attach";
        if (attach) {
            --argc;
            ++argv;
        }
        if (argc < 3)
            throw std::runtime_error("producer SHM DURATION [CONTROL_FILE]");
        std::signal(SIGTERM, Stop);
        std::signal(SIGINT, Stop);
        shm_utils::Mapping owner(argv[1], !attach);
        auto& shm = *owner.get();
        qhrr05::simulation_ipc::Server simulation_ipc(argv[1]);
        if (!attach)
            shm.can_enabled = true;
        const auto start = shm_utils::MonotonicNs();
        const double duration = std::stod(argv[2]);
        auto next = start;
        std::string command;
        while (!stopped && (shm_utils::MonotonicNs() - start) / 1e9 < duration) {
            if (argc == 4) {
                std::ifstream input(argv[3]);
                std::getline(input, command);
            }
            if (command == "pause") {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            simulation_ipc.Poll();
            SimulationData simulation;
            {
                shm_utils::Lock lock(shm);
                if (command == "hold")
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                const auto now = shm_utils::MonotonicNs();
                auto& motor = shm_utils::Next(shm.motors, shm.motor_sequence);
                motor.command_sequence++;
                motor.feedback_sequence++;
                motor.command_published_ns = motor.feedback_published_ns = now;
                motor.running = true;
                for (int i = 0; i < 3; ++i) {
                    motor.feedback[i].position = .1 * std::sin((now - start) / 1e9);
                    motor.feedback[i].velocity = .1 * std::cos((now - start) / 1e9);
                    motor.feedback[i].current = 2;
                    motor.motor_torque[i] = 2.88;
                    motor.command[i].position = motor.feedback[i].position + .1;
                    motor.received_ns[i] = now;
                    motor.received_sequence[i]++;
                }
                ++shm.motor_sequence;
                auto& model = shm_utils::Next(shm.model, shm.model_sequence);
                model.published_ns = now;
                model.joint_min.fill(-2);
                model.joint_max.fill(2);
                model.mass = {1, 0, 0, 0, 1, 0, 0, 0, 1};
                model.jacobian = model.mass;
                ++shm.model_sequence;
                auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                control.command_sequence++;
                control.published_ns = now;
                control.control_time = (now - start) / 1e9;
                control.mode = 2;
                control.magnetic.observer_valid = command != "invalid";
                control.magnetic.estimated_force = {0, 0, .9};
                ++shm.control_sequence;
                simulation.published_ns = now;
                simulation.time = control.control_time;
                simulation.contact_force = {0, 0, 1};
                simulation.contact = true;
                shm_utils::Notify(shm);
            }
            simulation_ipc.Publish(simulation);
            next += 2000000;
            const auto now = shm_utils::MonotonicNs();
            if (next <= now)
                next = now + 2000000;
            const timespec wake{
                static_cast<time_t>(next / 1000000000), static_cast<long>(next % 1000000000)};
            ::clock_nanosleep(kShmClock, TIMER_ABSTIME, &wake, nullptr);
        }
        simulation_ipc.Finish();
        shm_utils::Stop(shm);
        std::cout << "producer done\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
