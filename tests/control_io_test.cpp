#include "control/control.hpp"
#include "simulation/simulation.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

ControlData Latest(ShmData& shm) {
    shm_utils::Lock lock(shm);
    return shm.control[shm.control_sequence % kShmBufferCount];
}

MotorData Motors(ShmData& shm) {
    shm_utils::Lock lock(shm);
    return shm.motors[shm.motor_sequence % kShmBufferCount];
}

template <class Predicate> ControlData WaitCommand(ShmData& shm, Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);

    while (std::chrono::steady_clock::now() < deadline) {
        const auto command = Latest(shm);

        if (command.command_sequence && predicate(command))
            return command;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    throw std::runtime_error("test did not observe expected independent controller output");
}

std::uint64_t Request(ShmData& shm, int mode) {
    shm_utils::Lock lock(shm);
    auto& control = shm_utils::Next(shm.control, shm.control_sequence);
    control.requested_mode = mode;
    control.target_force = 3;
    ++control.motion_sequence;
    ++shm.control_sequence;
    shm_utils::Notify(shm);
    return control.motion_sequence;
}

void WaitExit(pid_t child, int expected) {
    int status = 0;

    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
    }

    Require(WIFEXITED(status) && WEXITSTATUS(status) == expected, "controller process failed");
}

int main(int argc, char** argv) {
    pid_t worker = -1;
    std::string temporary_directory;

    try {
        Require(argc == 4, "expected config, model and controller executable");
        char directory[] = "/tmp/qhrr05-independent-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        const auto path = temporary_directory + "/control.yaml";
        auto config = YAML::LoadFile(argv[1]);
        config["control"]["frequency_hz"] = 100;
        config["homing"]["duration_sec"] = .12;
        const auto save = [&] {
            std::ofstream output(path);
            output << config;
            output.close();
            Require(!output.fail(), "test config write failed");
        };

        save();
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity(
            mj_makeData(model.get()), mj_deleteData);
        const auto initial =
            qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        const auto prefix = "/qhrr05-independent-" + std::to_string(::getpid());
        const auto launch = [&](const std::string& name) {
            worker = ::fork();
            Require(worker >= 0, "fork failed");

            if (worker == 0) {
                ::execl(argv[3],
                    argv[3],
                    path.c_str(),
                    name.c_str(),
                    "--without-can",
                    static_cast<char*>(nullptr));
                ::_exit(127);
            }
        };

        {
            shm_utils::Mapping mapping(prefix, true);
            auto& shm = *mapping.get();
            launch(prefix);  // No simulator, not even an initial measurement.
            auto before = WaitCommand(shm, [](const ControlData&) {
                return true;
            });

            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            auto after = Latest(shm);
            const auto cycles = after.command_sequence - before.command_sequence;
            Require(
                cycles >= 8 && cycles <= 30, "configured 100 Hz loop is not running independently");
            Require(after.input_model_sequence == 0 && after.control_time > .1,
                "controller waited for simulator readiness");
            {
                shm_utils::Lock lock(shm);
                shm.simulation_finished = true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            Require(Latest(shm).command_sequence > after.command_sequence,
                "controller depends on simulator completion status");
            {
                shm_utils::Lock lock(shm);
                shm.simulation_finished = false;
            }

            qhrr05::ResetSimulation(initial, model.get(), data.get());
            std::array<double, 3> reference;
            data->time = 1000;  // Producer clock is unrelated to controller time.
            qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference);
            const auto request = Request(shm, 0);
            before = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == request;
            });

            for (int i = 0; i < 3; ++i)
                Require(std::abs(Motors(shm).reference[i] - initial[i]) < .02,
                    "explicit restart did not capture current position");

            after = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == request && cmd.holding;
            });

            Require(after.input_model_sequence == 1, "unchanged measurement was not reused");
            Require(after.control_time - before.control_time >= .1,
                "homing did not use controller elapsed time");

            for (int i = 0; i < 3; ++i)
                Require(std::abs(Motors(shm).reference[i] - home[i]) < 1e-12,
                    "homing stopped with a frozen simulator");

            for (int i = 0; i < 3; ++i)
                data->qvel[i] = .2 * (i + 1);

            qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference);
            const auto gravity_request = Request(shm, 1);
            after = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == gravity_request &&
                       cmd.input_model_sequence == 2;
            });

            for (int i = 0; i < 3; ++i) {
                const double damping = config["gravity_compensation"]["kd_motor"][i].as<double>();
                Require(std::abs(Motors(shm).debug_torque[i] -
                                 (gravity->qfrc_bias[i] - damping * data->qvel[i])) < 1e-10,
                    "gravity torque is wrong with frozen input");
            }

            const auto home_request = Request(shm, 0);
            WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == home_request && cmd.holding;
            });

            qhrr05::ResetSimulation(initial, model.get(), data.get());
            qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference);
            after = WaitCommand(shm, [](const ControlData& cmd) {
                return cmd.input_model_sequence == 3;
            });

            Require(after.holding, "simulation time rewind restarted the independent controller");
            const auto magnetic_request = Request(shm, 2);
            auto magnetic = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == magnetic_request && cmd.mode == 2;
            });

            Require(magnetic.magnetic.phase == MagneticPhase::Free,
                "magnetic mode without measurements did not remain free");
            const auto event = magnetic.magnetic.event_sequence;
            {
                shm_utils::Lock lock(shm);
                auto& request = shm_utils::Next(shm.control, shm.control_sequence);
                request.target_force = 4;
                ++shm.control_sequence;
                shm_utils::Notify(shm);
            }

            magnetic = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.magnetic.target_force == 4;
            });

            Require(magnetic.active_motion_sequence == magnetic_request &&
                        magnetic.magnetic.event_sequence == event,
                "live force update restarted the magnetic state machine");
            shm_utils::Stop(shm);
            WaitExit(worker, 0);
            worker = -1;
            Require(shm.control[shm.control_sequence % kShmBufferCount].command_sequence != 0,
                "commands were not published to double buffers");
            std::cout << "observed 100 Hz cycles in 150ms=" << cycles << '\n';
        }

        // A very slow configured period must still allow ordinary process shutdown.
        config["control"]["frequency_hz"] = .1;
        save();
        {
            shm_utils::Mapping mapping(prefix + "-slow", true);
            launch(prefix + "-slow");
            WaitCommand(*mapping.get(), [](const ControlData&) {
                return true;
            });

            const auto start = std::chrono::steady_clock::now();
            shm_utils::Stop(*mapping.get());
            WaitExit(worker, 0);
            worker = -1;
            Require(std::chrono::steady_clock::now() - start < std::chrono::seconds(1),
                "scheduler ignored stop request");
        }

        for (double frequency : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
            config["control"]["frequency_hz"] = frequency;
            save();
            shm_utils::Mapping mapping(prefix + "-invalid", true);
            launch(prefix + "-invalid");
            WaitExit(worker, 1);
            worker = -1;
        }

        // The simulator never waits for a controller, and holds any latest command
        // independently of how many states it has published.
        {
            shm_utils::Mapping mapping(prefix + "-viewer", true);
            auto& shm = *mapping.get();
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            std::array<double, 3> reference;
            Require(qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference)
                            .command_sequence == 0,
                "simulator did not accept initialized zero command");
            {
                shm_utils::Lock lock(shm);
                shm.control[0].command_sequence = 7;
                shm.motors[shm.motor_sequence % kShmBufferCount].debug_torque[0] = 1000;
            }

            const auto command =
                qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference);
            Require(command.command_sequence == 7 && shm.model_sequence == 2,
                "simulator still pairs state/command counters");
            Require(data->ctrl[0] == model->actuator_ctrlrange[1], "motor torque limit was lost");
            {
                shm_utils::Lock lock(shm);
                shm.motors[shm.motor_sequence % kShmBufferCount].debug_torque[0] =
                    std::numeric_limits<double>::quiet_NaN();
            }

            bool rejected = false;

            try {
                qhrr05::UpdateControlIO(shm, model.get(), data.get(), gravity.get(), reference);
            } catch (const std::runtime_error&) {
                rejected = true;
            }

            Require(rejected, "non-finite torque was accepted");
        }

        std::filesystem::remove_all(temporary_directory);
        std::cout
            << "PASS: configured independent loop without simulator, frozen feedback, independent clock/counters, explicit requests, no simulator wait\n";
        return 0;
    } catch (const std::exception& error) {
        if (worker > 0) {
            ::kill(worker, SIGKILL);

            while (::waitpid(worker, nullptr, 0) < 0 && errno == EINTR) {
            }
        }

        if (!temporary_directory.empty())
            std::filesystem::remove_all(temporary_directory);

        std::cerr << error.what() << '\n';

        return 1;
    }
}
