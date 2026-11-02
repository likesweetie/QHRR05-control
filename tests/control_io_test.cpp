#include "control/control.hpp"
#include "simulation/simulation.hpp"
#include "simulation_ipc_fixture.hpp"
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
        config["model_path"] = std::filesystem::absolute(argv[2]).string();
        config["control"]["frequency_hz"] = 100;
        config["homing"]["duration_sec"] = .12;
        config["gravity_compensation"]["friction_compensation"]["coulomb_ratio"] = .85;
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
            for (auto& m : shm.motors)
                m.coordinate_valid.fill(true);
            launch(prefix);  // No simulator, not even an initial measurement.
            auto before = WaitCommand(shm, [](const ControlData&) {
                return true;
            });
            Require(before.mode == 5 && !before.holding && !before.damping_reference_valid,
                "controller did not start in damping while awaiting its first model sample");

            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            auto after = Latest(shm);
            const auto cycles = after.command_sequence - before.command_sequence;
            Require(
                cycles >= 8 && cycles <= 30, "configured 100 Hz loop is not running independently");
            Require(after.input_model_sequence == 0 && after.control_time > .1,
                "controller waited for simulator readiness");
            SimulationData completed_simulation;
            completed_simulation.published_ns = shm_utils::MonotonicNs();
            completed_simulation.finished = true;
            SimulationIpcFixture simulation_finished(prefix, completed_simulation);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            Require(Latest(shm).command_sequence > after.command_sequence,
                "hardware control depends on simulator IPC completion status");

            std::uint64_t sample_sequence = 0;
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            std::array<double, 3> reference;
            data->time = 1000;  // Producer clock is unrelated to controller time.
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            qhrr05::InputInterface::RequestDrive(shm, DriveAction::Arm);
            // Publish again to acknowledge the enabled direct-effort device.
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                sample_sequence,
                nullptr,
                false);
            const auto damping = WaitCommand(shm, [](const ControlData& cmd) {
                return cmd.mode == 5 && cmd.input_model_sequence == 1 &&
                       cmd.damping_reference_valid;
            });
            const auto initial_toe = damping.damping_toe_reference;
            {
                // Legacy/external ModelData is not a trusted input to hardware control.
                shm_utils::Lock lock(shm);
                shm.model[shm.model_sequence % kShmBufferCount].gravity_torque.fill(
                    std::numeric_limits<double>::quiet_NaN());
            }
            const auto damping_motors = Motors(shm);

            for (int i = 0; i < 3; ++i) {
                Require(
                    std::abs(
                        initial_toe[i] -
                        data->site_xpos[3 * mj_name2id(model.get(), mjOBJ_SITE, "FL_foot") + i]) <
                        1e-12,
                    "damping did not capture the initial toe position");
                Require(damping_motors.debug_torque[i] == 0 &&
                            damping_motors.reference[i] == initial[i],
                    "damping applied position/gravity torque at rest");
                data->qpos[i] += .01;
                data->qvel[i] = .2 * (i + 1);
            }

            mj_forward(model.get(), data.get());
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            after = WaitCommand(shm, [](const ControlData& cmd) {
                return cmd.input_model_sequence == 2;
            });
            Require(after.mode == 5 && after.damping_toe_reference == initial_toe,
                "damping moved its captured toe reference");

            // The current damping controller uses the user's 20 Hz cutoff.
            const double sample_dt = model->opt.timestep;
            const double filtered_velocity = .01 / sample_dt *
                -std::expm1(-2 * 3.141592653589793 * 20 * sample_dt);
            for (int i = 0; i < 3; ++i)
                Require(std::abs(Motors(shm).debug_torque[i] + filtered_velocity) < 1e-9 &&
                            Motors(shm).command[i].velocity == 0 &&
                            Motors(shm).command[i].kp == 0 &&
                            Motors(shm).command[i].kd == 0 &&
                            Motors(shm).command[i].tau_ff == Motors(shm).debug_torque[i] &&
                            Motors(shm).reference[i] == data->qpos[i],
                    "damping did not filter position-derived velocity or send direct torque with zero gains");

            qhrr05::ResetSimulation(initial, model.get(), data.get());
            data->time = 1000;
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            const auto home_state_sequence = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.input_model_sequence == 3;
            }).input_model_sequence;
            std::cout
                << "PASS: startup damping, initial toe capture, zero-rest torque, pure velocity damping\n";
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

            Require(after.input_model_sequence == home_state_sequence,
                "unchanged measurement was not reused");
            Require(after.control_time - before.control_time >= .1,
                "homing did not use controller elapsed time");

            for (int i = 0; i < 3; ++i)
                Require(std::abs(Motors(shm).reference[i] - home[i]) < 1e-12,
                    "homing stopped with a frozen simulator");
            const auto home_motors = Motors(shm);
            for (int i = 0; i < 3; ++i) {
                const double kp = config["homing"]["kp"][i].as<double>();
                const double error_limit = config["limits"]["torque_nm"][i].as<double>() / kp;
                const double expected = std::clamp(home[i],
                    initial[i] - error_limit, initial[i] + error_limit);
                Require(std::abs(home_motors.command[i].position - expected) < 1e-12 &&
                            home_motors.debug_torque[i] == 0,
                    "homing position target was dropped or mixed into torque");
            }

            for (int i = 0; i < 3; ++i)
                data->qvel[i] = .2 * (i + 1);

            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            const auto gravity_request = Request(shm, 1);
            after = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == gravity_request &&
                       cmd.input_model_sequence == home_state_sequence + 1;
            });

            mj_copyData(gravity.get(), model.get(), data.get());
            mju_zero(gravity->qvel, model->nv);
            mj_forward(model.get(), gravity.get());
            for (int i = 0; i < 3; ++i) {
                const double drag_compensation =
                    .85 * model->dof_frictionloss[i] * std::tanh(data->qvel[i] / .03);
                Require(std::abs(Motors(shm).debug_torque[i] -
                                 (gravity->qfrc_bias[i] + drag_compensation)) < 1e-10,
                    "gravity torque is wrong with frozen input");
                const auto command = Motors(shm).command[i];
                Require(std::abs(command.kp * (command.position - data->qpos[i]) -
                            Motors(shm).debug_torque[i]) < 1e-10,
                    "gravity torque was not added once as an external position offset");
            }

            const auto home_request = Request(shm, 0);
            WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.active_motion_sequence == home_request && cmd.holding;
            });

            qhrr05::ResetSimulation(initial, model.get(), data.get());
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            after = WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.input_model_sequence == home_state_sequence + 2;
            });

            Require(after.holding, "simulation time rewind restarted the independent controller");
            // A second steady sample clears the earlier finite-difference acceleration.
            qhrr05::UpdateControlIO(shm,
                model.get(),
                data.get(),
                gravity.get(),
                reference,
                ++sample_sequence,
                nullptr,
                false);
            WaitCommand(shm, [&](const ControlData& cmd) {
                return cmd.input_model_sequence == sample_sequence;
            });
            {
                // A balanced measured effort gives zero external force with frozen encoders.
                shm_utils::Lock lock(shm);
                std::copy_n(data->qfrc_bias,
                    3,
                    shm.motors[shm.motor_sequence % kShmBufferCount].motor_torque.begin());
            }
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

        // Compare both output encodings, including independent position + torque
        // and saturated effort. Feedback stays in this test-owned SHM mapping.
        const auto normal_limits = YAML::Clone(config["limits"]);
        const auto normal_config = YAML::Clone(config);
        for (const std::string variant :
            {"torque_ref_as_position", "native_torque_ref", "native_zero_kp", "rmd_torque_ref",
                "position_homing_torque", "native_homing_torque", "rmd_homing_torque",
                "native_homing_torque_difference"}) {
            const bool zero_kp = variant == "native_zero_kp";
            const bool homing_torque = variant.find("homing_torque") != std::string::npos;
            const bool differentiate_homing = variant == "native_homing_torque_difference";
            const std::string torque_mode = zero_kp || variant == "native_homing_torque" || differentiate_homing
                ? "native_torque_ref" : variant == "position_homing_torque"
                ? "torque_ref_as_position" : variant == "rmd_homing_torque"
                ? "rmd_torque_ref" : variant;
            for (bool limited : {false, true}) {
                config = YAML::Clone(normal_config);
                config["torque_reference_mode"] = torque_mode;
                config["homing"]["torque_control"] = homing_torque;
                if (differentiate_homing) {
                    config["homing"]["differentiate_position"] = true;
                    config["homing_velocity_lpf"]["enabled"] = true;
                    config["homing_velocity_lpf"]["cutoff_hz"] = 20;
                }
                if (zero_kp) {
                    config["homing"]["kp"] = YAML::Load("[0,0,0]");
                    config["gravity_compensation"]["kp_motor"] = YAML::Load("[0,40,0]");
                    config["magnetic_touch"]["kp_motor"] = YAML::Load("[0,0,0]");
                    config["tracking"]["motor"]["kp_nm_rad"] = YAML::Load("[0,0,0]");
                    config["ball_following"]["kp_motor"] = YAML::Load("[0,0,0]");
                }
                if (limited)
                    config["limits"]["torque_nm"] = .1;
                save();
                const auto name = prefix + "-" + variant + (limited ? "-limited" : "-normal");
                shm_utils::Mapping mapping(name, true);
                auto& shm = *mapping.get();
                shm.can_enabled = true;
                auto& feedback = shm.motors[0];
                feedback.running = true;
                feedback.supported.fill(kCanSignals);
                feedback.valid.fill(kCanSignals);
                feedback.coordinate_valid.fill(true);
                feedback.enabled.fill(true);
                feedback.measurement_ns.fill(shm_utils::MonotonicNs());
                feedback.received_sequence.fill(1);
                feedback.received_ns.fill(shm_utils::MonotonicNs());
                feedback.profile_verified.fill(true);
                for (int i = 0; i < 3; ++i) {
                    feedback.feedback[i].position = home[i];
                    feedback.feedback[i].velocity = 2 + i;
                }
                qhrr05::TaskController controller(path, name);
                qhrr05::InputInterface::RequestDrive(shm, DriveAction::Arm);
                bool saw_position_and_torque = false, saw_saturation = false;
                for (int mode : {5, 0, 1, 4}) {
                    Request(shm, mode);
                    Require(controller.ReadInput(), "controller refused valid feedback");
                    controller.Control();
                    controller.WriteOutput();
                    const auto before_move = Motors(shm);
                    if (mode == 5) {
                        {
                            shm_utils::Lock lock(shm);
                            auto& moved = shm_utils::Next(shm.motors, shm.motor_sequence);
                            for (int i = 0; i < 3; ++i) {
                                moved.feedback[i].position += .001;
                                ++moved.received_sequence[i];
                                moved.measurement_ns[i] += 1000000;
                            }
                            ++shm.motor_sequence;
                        }
                    }
                    // Move the measured joints after capturing the position reference.
                    // Ball now has a position error AND a nonzero gravity torque.
                    if (mode == 0 || mode == 4) {
                        shm_utils::Lock lock(shm);
                        auto& moved = shm_utils::Next(shm.motors, shm.motor_sequence);
                        const std::array<double, 3> delta{.06, -.05, .04};
                        for (int i = 0; i < 3; ++i) {
                            moved.feedback[i].position += delta[i];
                            ++moved.received_sequence[i];
                            moved.received_ns[i] += 1000000;
                            moved.measurement_ns[i] += 1000000;
                        }
                        ++shm.motor_sequence;
                    }
                    Require(controller.ReadInput(), "controller refused moved feedback");
                    controller.Control();
                    controller.WriteOutput();
                    const auto motors = Motors(shm);
                    for (int i = 0; i < 3; ++i) {
                        const auto& command = motors.command[i];
                        const bool host_pd = homing_torque && mode == 0;
                        const bool rmd = torque_mode == "rmd_torque_ref" && (host_pd || mode == 1 || mode == 5);
                        Require(command.rmd_torque == rmd, "incorrect RMD/MIT routing for motion");
                        Require(command.rmd_damping_constant == (rmd ? 5 : 0),
                            "RMD default damping was missing or leaked into MIT commands");
                        const double position = motors.feedback[i].position;
                        const double limit =
                            limited ? .1 : normal_limits["torque_nm"][i].as<double>();
                        const double requested_effort =
                            (host_pd ? 0 : command.kp * (motors.reference[i] - position)) + motors.debug_torque[i];
                        if (host_pd) {
                            double velocity = motors.feedback[i].velocity;
                            if (differentiate_homing) {
                                const double dt = (motors.measurement_ns[i] - before_move.measurement_ns[i]) / 1e9;
                                velocity = (position - before_move.feedback[i].position) / dt *
                                    -std::expm1(-2 * 3.14159265358979323846 * 20 * dt);
                            }
                            const double expected = config["homing"]["kp"][i].as<double>() *
                                (motors.reference[i] - position) -
                                config["homing"]["kd"][i].as<double>() * velocity;
                            Require(std::abs(motors.debug_torque[i] - std::clamp(expected, -limit, limit)) < 1e-10,
                                "homing PD torque did not reach the output exactly once");
                        }
                        const double effort =
                            command.kp * (command.position - position) + command.tau_ff;
                        Require(std::abs(motors.debug_torque[i]) <= limit + 1e-10,
                            "software torque target bypassed shared torque limit");
                        Require(command.velocity == 0, "MIT output introduced a velocity reference");
                        Require(std::abs(effort - std::clamp(requested_effort, -limit, limit)) < 1e-10,
                            "MIT output lost or duplicated position/torque, or bypassed the effort limit");
                        if (mode == 5) {
                            Require(command.kp == 0 && command.kd == 0 &&
                                        command.tau_ff == motors.debug_torque[i] && command.tau_ff < 0,
                                "output mode changed the direct-torque damping test");
                        } else if (rmd) {
                            Require(command.kp == 0 && command.kd == 0 &&
                                        command.tau_ff == motors.debug_torque[i],
                                "RMD must carry pure torque without implicit position/velocity feedback");
                        } else {
                            const auto gains = mode == 0   ? config["homing"]
                                               : mode == 1 ? config["gravity_compensation"]
                                                           : config["ball_following"];
                            const double expected_kp = host_pd && torque_mode != "torque_ref_as_position"
                                ? 0 : gains[mode == 0 ? "kp" : "kp_motor"][i].as<double>();
                            const double expected_kd = host_pd
                                ? 0 : gains[mode == 0 ? "kd" : "kd_motor"][i].as<double>();
                            Require(command.kp == expected_kp && command.kd == expected_kd,
                                "output mode changed the motor P/D gains");
                            Require(command.tau_ff == (torque_mode != "torque_ref_as_position"
                                                          ? motors.debug_torque[i]
                                                          : 0),
                                "torque reference was sent through the wrong MIT field");
                            if (torque_mode != "torque_ref_as_position" && std::abs(requested_effort) < limit)
                                Require(command.position == motors.reference[i],
                                    "native torque output altered an unsaturated position reference");
                            saw_position_and_torque |=
                                mode == 4 && std::abs(motors.reference[i] - position) > .001 &&
                                std::abs(motors.debug_torque[i]) > .01;
                            saw_saturation |= std::abs(requested_effort) > limit;
                        }
                    }
                    if (mode == 5) {
                        {
                            shm_utils::Lock lock(shm);
                            auto& invalid = shm_utils::Next(shm.motors, shm.motor_sequence);
                            invalid.valid[0] &= ~EncoderPosition;
                            ++shm.motor_sequence;
                        }
                        Require(controller.ReadInput(), "invalid encoder stopped input handling");
                        controller.Control();
                        controller.WriteOutput();
                        for (const auto& output : Motors(shm).command)
                            Require(output.tau_ff == 0, "Damping kept torque with invalid MIT position");
                        {
                            shm_utils::Lock lock(shm);
                            auto& restored = shm_utils::Next(shm.motors, shm.motor_sequence);
                            restored.valid[0] |= EncoderPosition;
                            ++shm.motor_sequence;
                        }
                    }
                }
                Require(saw_position_and_torque,
                    "test did not exercise independent position and torque");
                Require(!limited || zero_kp || saw_saturation,
                    "test did not exercise combined effort saturation");
            }
        }
        std::cout
            << "PASS: both torque modes preserve position/P/D, direct damping and combined effort limits\n";
        config = YAML::Clone(normal_config);

        // Reject a zero-gain position conversion before any control cycle starts.
        config["homing"]["kp"] = YAML::Load("[40,0,40]");
        save();
        {
            const auto name = prefix + "-invalid-zero-kp";
            shm_utils::Mapping mapping(name, true);
            bool rejected = false;
            try {
                qhrr05::TaskController controller(path, name, true);
            } catch (const std::exception& error) {
                rejected = std::string(error.what()).find("homing.kp") != std::string::npos;
            }
            Require(rejected, "TaskController started with kp=0 in position-torque mode");
            Require(Latest(*mapping.get()).command_sequence == 0,
                "invalid position-torque configuration published a motor command");
        }
        config = YAML::Clone(normal_config);

        for (const auto invalid :
            {YAML::Node(), YAML::Node("unknown"), YAML::Load("[native_torque_ref]")}) {
            config["torque_reference_mode"] = invalid;
            save();
            const auto name = prefix + "-invalid-torque-mode";
            shm_utils::Mapping mapping(name, true);
            bool rejected = false;
            try {
                qhrr05::TaskController controller(path, name, true);
            } catch (const std::exception& error) {
                rejected =
                    std::string(error.what()).find("torque_reference_mode") != std::string::npos;
            }
            Require(rejected, "invalid/missing torque reference mode was not rejected clearly");
        }
        config["torque_reference_mode"] = "torque_ref_as_position";

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
            Require(qhrr05::UpdateControlIO(
                        shm, model.get(), data.get(), gravity.get(), reference, 0, nullptr, true)
                            .command_sequence == 0,
                "simulator did not accept initialized zero command");
            {
                shm_utils::Lock lock(shm);
                shm.control[0].command_sequence = 7;
                auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];
                motors.debug_torque[0] = -123;  // Diagnostics must not actuate the plant.
                motors.command[0] = {initial[0] + 1000, 0, 1, 0, 0};
                shm.motors[shm.motor_sequence % kShmBufferCount].command_enabled[0] = true;
            }

            const auto command = qhrr05::UpdateControlIO(
                shm, model.get(), data.get(), gravity.get(), reference, 0, nullptr, true);
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
                qhrr05::UpdateControlIO(
                    shm, model.get(), data.get(), gravity.get(), reference, 0, nullptr, true);
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
