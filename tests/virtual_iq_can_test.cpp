#include "simulation/simulation.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/wait.h>
#include <thread>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    pid_t hal = -1;
    std::unique_ptr<shm_utils::Mapping> owner;
    std::filesystem::path directory;
    std::string name;

    try {
        Require(argc == 5, "expected scene, HAL config, simulation config and HAL executable");
        directory = "/tmp/qhrr05-iq-can-" + std::to_string(getpid());
        std::filesystem::create_directory(directory);
        const auto hal_path = (directory / "hal.yaml").string();
        const auto sim_path = (directory / "simulation.yaml").string();
        auto hal_config = YAML::LoadFile(argv[2]);
        hal_config["motor_config"] =
            std::filesystem::absolute(std::filesystem::path(argv[2]).parent_path() /
                                      hal_config["motor_config"].as<std::string>())
                .string();
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        const std::array<double, 3> origin{0, .3, -1};
        const MitCommand command{.1, 0, 20, .3, 0};
        const auto decoded_command =
            openrobot::DecodeMitCommand(openrobot::EncodeMitCommand(command, 45, 1), 45, 1);

        for (int trial = 0; trial < 4; ++trial) {
            const bool enabled = trial != 0;
            const bool noisy = trial == 3;
            auto settings = YAML::LoadFile(argv[3]);
            settings["iq_sensor"]["enabled"] = enabled;
            settings["iq_sensor"]["offset_a"] = .4;
            settings["iq_sensor"]["noise_std_a"] = noisy ? .03 : 0;
            settings["iq_sensor"]["ripple_amplitude_a"] = 0;
            settings["iq_sensor"]["bandwidth_hz"] = 0;
            settings["iq_sensor"]["quantization_a"] = 0;

            if (trial == 2)
                settings["motor"]["kt_motor_nm_a"] = YAML::Load("[.08,.08,.08]");

            std::ofstream(sim_path) << settings;
            name = "/qhrr05-iq-can-" + std::to_string(getpid()) + "-" + std::to_string(trial);
            std::ofstream(hal_path) << hal_config;
            owner = std::make_unique<shm_utils::Mapping>(name, true);
            owner->get()->can_enabled = true;

            hal = ::fork();
            Require(hal >= 0, "HAL fork failed");

            if (!hal) {
                ::execl(
                    argv[4], argv[4], hal_path.c_str(), name.c_str(), static_cast<char*>(nullptr));
                ::_exit(127);
            }

            shm_utils::Mapping mapping(name);
            auto& shm = *mapping.get();
            const auto startup = std::chrono::steady_clock::now() + std::chrono::seconds(3);

            for (;;) {
                {
                    shm_utils::Lock lock(shm);

                    if (shm.motors[shm.motor_sequence % kShmBufferCount].running)
                        break;
                }

                Require(std::chrono::steady_clock::now() < startup, "HAL did not initialize");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            qhrr05::ResetSimulation(origin, model.get(), data.get());
            qhrr05::VirtualMotors motors(hal_path, sim_path, origin, shm);
            const auto snapshot = [&] {
                shm_utils::Lock lock(shm);
                return shm.motors[shm.motor_sequence % kShmBufferCount];
            };
            const auto service = [&](auto predicate) {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);

                while (!predicate()) {
                    motors.Update(model.get(), data.get());
                    Require(
                        std::chrono::steady_clock::now() < deadline, "CAN sensor trial timed out");
                    int status;
                    Require(::waitpid(hal, &status, WNOHANG) == 0, "HAL exited during sensor test");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            };
            service([&] {
                const auto state = snapshot();
                return state.profile_verified[0] && state.profile_verified[1] &&
                       state.profile_verified[2];
            });
            {
                shm_utils::Lock lock(shm);
                auto& targets = shm_utils::Next(shm.motors, shm.motor_sequence);
                targets.command.fill(command);
                targets.command_enabled.fill(true);
                ++targets.command_sequence;
                ++shm.motor_sequence;
            }

            service([&] {
                return snapshot().received_sequence[2] >= 2;
            });

            for (int step = 0; step < 80; ++step) {
                motors.Update(model.get(), data.get());
                mj_step(model.get(), data.get());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            auto before = snapshot();
            service([&] {
                return snapshot().received_sequence[2] >= before.received_sequence[2] + 5;
            });
            const auto measured = snapshot();

            for (int axis = 0; axis < 3; ++axis) {
                const double actual_torque = data->qfrc_actuator[axis];
                const double actual_kt = settings["motor"]["kt_motor_nm_a"][axis].as<double>() * 9;
                const double expected_iq =
                    enabled ? actual_torque / actual_kt + .4 : actual_torque / 1.44;
                const double tolerance = noisy ? .2 : 33.0 / 2048;
                Require(std::abs(measured.feedback[axis].current - expected_iq) <= tolerance,
                    "CAN Iq did not reflect sensor/current-model configuration");
                Require(std::abs(measured.motor_torque[axis] -
                                 measured.feedback[axis].current * 1.44) < 1e-12,
                    "HAL did not calculate torque from the actual returned CAN Iq");
                const double servo =
                    decoded_command.kp *
                        (decoded_command.position - (data->qpos[axis] - origin[axis])) +
                    decoded_command.kd * (decoded_command.velocity - data->qvel[axis]);
                Require(std::abs(data->ctrl[axis] - std::clamp(servo,
                                                        model->actuator_ctrlrange[2 * axis],
                                                        model->actuator_ctrlrange[2 * axis + 1])) <
                            1e-10,
                    "sensor error changed physical servo torque");
            }

            before = measured;
            service([&] {
                return snapshot().received_sequence[2] >= before.received_sequence[2] + 5;
            });
            const auto paused = snapshot();

            for (int axis = 0; axis < 3; ++axis)
                Require(paused.feedback[axis].current == measured.feedback[axis].current,
                    "repeated CAN requests resampled a paused sensor");

            if (enabled) {
                qhrr05::ResetSimulation(origin, model.get(), data.get());
                before = snapshot();
                service([&] {
                    return snapshot().received_sequence[2] >= before.received_sequence[2] + 5;
                });
                const auto reset = snapshot();

                for (int axis = 0; axis < 3; ++axis) {
                    qhrr05::VirtualIqSensor expected(sim_path, axis);
                    const double angle = 9 * origin[axis];
                    const double iq = expected.Sample(0, angle, model->opt.timestep);
                    const auto wire =
                        openrobot::DecodeFeedback(openrobot::EncodeFeedback({0, 0, iq, 25}));
                    Require(reset.feedback[axis].current == wire.current,
                        "simulation reset did not reset sensor state/noise seed");
                }
            }

            ::kill(hal, SIGTERM);
            int status = 0;
            Require(
                ::waitpid(hal, &status, 0) == hal && WIFEXITED(status) && WEXITSTATUS(status) == 0,
                "HAL sensor trial shutdown failed");
            hal = -1;
            Require(std::filesystem::exists("/dev/shm/" + name.substr(1)),
                "HAL unlinked daemon-owned SHM");
            owner.reset();
            Require(
                !std::filesystem::exists("/dev/shm/" + name.substr(1)), "owner left SHM behind");
            std::cout << "PASS: Iq CAN trial " << trial
                      << " bypass/bias/Kt/noise, torque separation, pause/reset\n";
        }

        std::filesystem::remove_all(directory);
        return 0;
    } catch (const std::exception& error) {
        if (hal > 0) {
            ::kill(hal, SIGKILL);
            ::waitpid(hal, nullptr, 0);
            ::shm_unlink(name.c_str());
        }

        if (!directory.empty())
            std::filesystem::remove_all(directory);

        std::cerr << error.what() << '\n';
        return 1;
    }
}
