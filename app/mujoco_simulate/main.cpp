#include <array>
#include "simulation.hpp"
#include "daemon/daemon.hpp"
#include "telemetry.hpp"
#include "simulate/simulate.h"
#include "simulate/glfw_adapter.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;

void OnSignal(int) {
    stopped = 1;
}

struct Options {
    std::string model, config, shm_name, hal_config, simulation_config;
    bool without_can = false;
    bool headless = false;
    double duration = 0;
};

Options Parse(int argc, char** argv) {
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--without-can") {
            options.without_can = true;
            continue;
        }

        if (arg == "--headless") {
            options.headless = true;
            continue;
        }

        if (i + 1 >= argc)
            throw std::runtime_error("missing value for " + arg);

        const std::string value = argv[++i];

        if (arg == "--model")
            options.model = value;
        else if (arg == "--control-config")
            options.config = value;
        else if (arg == "--shm-name")
            options.shm_name = value;
        else if (arg == "--hal-config")
            options.hal_config = value;
        else if (arg == "--simulation-config")
            options.simulation_config = value;
        else if (arg == "--duration") {
            std::size_t count = 0;
            options.duration = std::stod(value, &count);

            if (count != value.size() || !std::isfinite(options.duration) || options.duration <= 0)
                throw std::runtime_error("duration must be positive and finite");
        } else
            throw std::runtime_error("unknown viewer argument: " + arg);
    }

    if (options.config.empty() || options.model.empty() || options.shm_name.empty())
        throw std::runtime_error("--model, --control-config and --shm-name are required");

    if (options.headless && options.duration <= 0)
        throw std::runtime_error("--headless requires --duration");

    if ((options.without_can && !options.hal_config.empty()) ||
        (!options.without_can && options.hal_config.empty()))
        throw std::runtime_error("select --hal-config YAML or explicit --without-can");

    if ((options.without_can && !options.simulation_config.empty()) ||
        (!options.without_can && options.simulation_config.empty()))
        throw std::runtime_error(
            "--simulation-config is required in CAN mode and unavailable with --without-can");

    return options;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try {
        const auto options = Parse(argc, argv);

        const auto config = YAML::LoadFile(options.config);

        if (config["robot"].as<std::string>() != "qhrr-05")
            throw std::runtime_error("only qhrr-05 is supported");

        const double sample_hz = config["plots"]["sample_hz"].as<double>();
        const double history_sec = config["plots"]["history_sec"].as<double>();

        if (!std::isfinite(sample_hz) || !std::isfinite(history_sec) || sample_hz <= 0 ||
            history_sec <= 0 || sample_hz * history_sec > mjMAXLINEPNT)
            throw std::runtime_error("invalid plot sampling or history length");

        shm_utils::Mapping mapping(options.shm_name);
        auto& shm = *mapping.get();

        char error[2048] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(options.model.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);

        if (!model)
            throw std::runtime_error(std::string("model load failed: ") + error);

        if (model->nq != 3 || model->nv != 3 || model->nu != 3)
            throw std::runtime_error("expected three joint positions, velocities and motor inputs");

        const auto initial =
            qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());

        if (sample_hz > 1.0 / model->opt.timestep)
            throw std::runtime_error("plot rate exceeds simulation rate");

        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity_data(
            mj_makeData(model.get()), mj_deleteData);

        if (!data || !gravity_data)
            throw std::runtime_error("could not allocate model data");

        qhrr05::ResetSimulation(initial, model.get(), data.get());

        {
            shm_utils::Lock lock(shm);

            if (shm.can_enabled == options.without_can)
                throw std::runtime_error("SHM transport does not match simulator mode");

            auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);

            for (int i = 0; i < 3; ++i)
                motors.position_offset[i] = initial[i];

            ++shm.motor_sequence;
        }

        std::uint64_t run_id = 0;
        auto plots = std::make_unique<qhrr05::viewer::LivePlots>(history_sec);
        std::array<double, 3> reference = initial;
        std::uint64_t measurement_sequence = 0;

        std::unique_ptr<qhrr05::VirtualMotors> virtual_motors;
        std::unique_ptr<mjData, decltype(&mj_deleteData)> model_data(nullptr, mj_deleteData);

        if (!options.without_can) {
            virtual_motors = std::make_unique<qhrr05::VirtualMotors>(
                options.hal_config, options.simulation_config, initial, shm);
            model_data.reset(mj_makeData(model.get()));

            if (!model_data)
                throw std::runtime_error("could not allocate CAN model state");
        }

        const auto update_io = [&] {
            if (!virtual_motors)
                return qhrr05::UpdateControlIO(shm,
                    model.get(),
                    data.get(),
                    gravity_data.get(),
                    reference,
                    measurement_sequence);

            const auto feedback = virtual_motors->ReadFeedback();

            if (!feedback.running) {
                stopped = 1;
                throw std::runtime_error("CAN HAL stopped normally");
            }

            // Model terms use decoded encoders, not the simulator's physical state.
            for (int i = 0; i < 3; ++i) {
                model_data->qpos[i] = feedback.position_offset[i] + feedback.feedback[i].position;
                model_data->qvel[i] = feedback.feedback[i].velocity;
            }

            return qhrr05::UpdateControlIO(
                shm, model.get(), model_data.get(), gravity_data.get(), reference, 0, &feedback);
        };

        const int object_body = mj_name2id(model.get(), mjOBJ_BODY, "touch_object");

        if (object_body < 0)
            throw std::runtime_error("missing touch_object body");

        const int object_mocap = model->body_mocapid[object_body];
        const int toe_site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");

        if (toe_site < 0)
            throw std::runtime_error("missing FL_foot site");

        double target_force = config["magnetic_touch"]["target_force_n"].as<double>();
        const double maximum_force = config["magnetic_touch"]["maximum_force_n"].as<double>();

        ControlData applied_command = update_io();

        std::uint64_t request_sequence = 0;
        int mode = 0;  // Mutually exclusive: homing or gravity compensation.
        double next_sample = 0;
        bool holding_reported = false;

        const auto request_control = [&](int requested) {
            shm_utils::Lock lock(shm);
            auto& control = shm_utils::Next(shm.control, shm.control_sequence);
            control.requested_mode = requested;
            control.target_force = target_force;
            ++control.motion_sequence;
            ++shm.control_sequence;
            shm_utils::Notify(shm);
            return control.motion_sequence;
        };

        const auto reset = [&] {
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            ++run_id;
            plots->Clear();

            reference = initial;
            next_sample = 0;
            holding_reported = false;
            mode = 0;

            request_sequence = request_control(mode);
            applied_command = update_io();
        };

        const auto select_controller = [&](int requested) {
            if (requested == mode)
                return;

            mode = requested;
            request_sequence = request_control(mode);
            holding_reported = false;

            std::cout << "[control] "
                      << (mode == 0 ? "HOMING" : (mode == 1 ? "GRAVITY" : "MAGNETIC"))
                      << " at t=" << data->time << std::endl;
        };

        const auto sample = [&] {
            qhrr05::viewer::Sample frame;
            frame.time = data->time;
            frame.run_id = run_id;
            frame.mode = applied_command.mode;
            frame.control_time = applied_command.control_time;
            frame.reference = reference;
            frame.magnetic = applied_command.magnetic;

            std::copy_n(data->mocap_pos + 3 * object_mocap, 3, frame.object_position.begin());
            const auto ground_truth = qhrr05::ReadContactGroundTruth(model.get(), data.get());
            std::copy_n(data->site_xpos + 3 * toe_site, 3, frame.toe_position.begin());
            std::copy_n(ground_truth.begin(), 3, frame.contact_force.begin());
            std::copy_n(ground_truth.begin() + 3, 3, frame.contact_torque.begin());

            std::copy_n(data->qpos, 3, frame.position.begin());

            {
                shm_utils::Lock lock(shm);
                frame.measured_torque =
                    shm.motors[shm.motor_sequence % kShmBufferCount].motor_torque;
                auto& snapshot = shm_utils::Next(shm.simulation, shm.simulation_sequence);
                snapshot.run_id = frame.run_id;
                snapshot.time = frame.time;
                snapshot.position_gt = frame.position;
                std::copy_n(data->actuator_force, 3, snapshot.torque_gt.begin());
                snapshot.toe_position = frame.toe_position;
                snapshot.contact_force = frame.contact_force;
                snapshot.contact_torque = frame.contact_torque;
                snapshot.object_position = frame.object_position;
                const bool contact = mju_norm3(frame.contact_force.data()) > 0;

                if (contact != snapshot.contact) {
                    ++snapshot.contact_event_sequence;
                    snapshot.contact_event_time = frame.time;
                }

                snapshot.contact = contact;
                snapshot.published_ns = shm_utils::MonotonicNs();
                ++shm.simulation_sequence;
                shm_utils::Notify(shm);
            }

            plots->Push(frame);
        };

        const auto check_holding = [&] {
            if (!holding_reported && mode == 0 && applied_command.mode == 0 &&
                applied_command.active_motion_sequence == request_sequence &&
                applied_command.holding) {
                std::cout << "[homing] HOLD: maintaining configured homing pose" << std::endl;
                holding_reported = true;
            }
        };

        const auto step = [&] {
            applied_command = update_io();

            if (virtual_motors)
                virtual_motors->Update(model.get(), data.get());

            mj_step(model.get(), data.get());

            ++measurement_sequence;
            mj_forward(model.get(), data.get());

            for (int i = 0; i < mjNWARNING; ++i)
                if (data->warning[i].number)
                    throw std::runtime_error("MuJoCo dynamics warning");

            if (data->time + 1e-9 >= next_sample) {
                sample();
                next_sample = data->time + 1.0 / sample_hz;
            }

            check_holding();
        };

        const auto finished = [&] {
            return options.duration > 0 && data->time + 1e-9 >= options.duration;
        };

        std::cout << "[simulate] "
                  << (virtual_motors ? "MIT CAN virtual motors" : "DEBUG direct SHM torque")
                  << " + independent control superloop" << std::endl;
        Daemon::NotifyReady();

        if (options.headless) {
            auto next_tick = std::chrono::steady_clock::now();

            while (!stopped && !finished()) {
                step();

                if (virtual_motors) {
                    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(model->opt.timestep));
                    std::this_thread::sleep_until(next_tick);
                }
            }
        } else {
            mjvCamera camera;
            mjv_defaultCamera(&camera);
            mjvOption visual;
            mjv_defaultOption(&visual);
            mjvPerturb perturb;
            mjv_defaultPerturb(&perturb);

            auto sim = std::make_unique<mujoco::Simulate>(
                std::make_unique<mujoco::GlfwAdapter>(), &camera, &visual, &perturb, false);
            std::atomic<bool> failed{false};

            sim->user_panel_percent_ = 40;
            sim->user_target_force_ = target_force;
            sim->user_maximum_force_ = maximum_force;

            // Keyboard events and all model mutations use simulate's existing mutex.
            const auto guarded = [&](auto operation) {
                try {
                    operation();
                } catch (const std::exception& exception) {
                    std::cerr << "[simulate] ERROR: " << exception.what() << std::endl;
                    failed = true;
                    sim->exitrequest.store(1);
                }
            };

            sim->user_reset_callback_ = [&] {
                guarded(reset);
            };

            sim->user_step_callback_ = [&] {
                guarded(step);
            };

            sim->user_controller_callback_ = [&](int requested) {
                const mujoco::MutexLock lock(sim->mtx);

                guarded([&] {
                    select_controller(requested);
                });
            };

            sim->user_force_callback_ = [&](double requested) {
                const mujoco::MutexLock lock(sim->mtx);

                guarded([&] {
                    if (!std::isfinite(requested) || requested < 0 || requested > maximum_force)
                        throw std::runtime_error("invalid target force");

                    target_force = requested;

                    shm_utils::Lock shared_lock(shm);
                    auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                    control.target_force = target_force;
                    ++shm.control_sequence;
                    shm_utils::Notify(shm);
                });
            };

            sim->user_key_callback_ = [&](int key) {
                if (key == mjKEY_ESCAPE) {
                    sim->exitrequest.store(1);
                    return true;
                }

                if (key == 'R' || key == 'r') {
                    sim->pending_.reset = true;
                    return true;
                }

                if (key == 'H' || key == 'h') {
                    sim->user_controller_callback_(0);
                    return true;
                }

                if (key == 'G' || key == 'g') {
                    sim->user_controller_callback_(1);
                    return true;
                }

                if (key == 'M' || key == 'm') {
                    sim->user_controller_callback_(2);
                    return true;
                }

                return false;
            };

            sim->user_render_callback_ = [&] {
                const mujoco::MutexLock lock(sim->mtx);

                if (stopped)
                    sim->exitrequest.store(1);

                const auto rect = sim->uistate.rect[3];
                const int width = rect.width * sim->user_panel_percent_ / 100;
                plots->Draw({rect.left + rect.width - width, rect.bottom, width, rect.height},
                    &sim->platform_ui->mjr_context());

                double max_error = 0;

                for (int i = 0; i < 3; ++i)
                    max_error = std::max(max_error, std::abs(home[i] - data->qpos[i]));

                char status[512];
                std::snprintf(status,
                    sizeof(status),
                    "%s%s | sim=%.2f s ctrl=%.2f s | home error=%.4f rad",
                    mode == 2 ? MagneticPhaseName(applied_command.magnetic.phase)
                              : (mode == 1 ? "GRAVITY" : (holding_reported ? "HOLD" : "HOMING")),
                    !sim->run ? " / PAUSED" : "",
                    data->time,
                    applied_command.control_time,
                    max_error);

                if (mode == 2) {
                    const auto& magnetic = applied_command.magnetic;
                    const auto truth = qhrr05::ReadContactGroundTruth(model.get(), data.get());
                    const double moment =
                        std::sqrt(truth[3] * truth[3] + truth[4] * truth[4] + truth[5] * truth[5]);

                    std::snprintf(status,
                        sizeof(status),
                        "%s%s | sim=%.2f ctrl=%.2f\nFhat=%.2f target=%.2f cmd=%.2f N\n%s | moment GT=%.4f Nm | events=%llu",
                        MagneticPhaseName(magnetic.phase),
                        !sim->run ? " / PAUSED" : "",
                        data->time,
                        applied_command.control_time,
                        magnetic.estimated_normal_force,
                        target_force,
                        magnetic.force_command,
                        MagneticReasonName(magnetic.reason),
                        moment,
                        static_cast<unsigned long long>(magnetic.event_sequence));
                }

                const mjrRect scene_rect{rect.left, rect.bottom, rect.width - width, rect.height};
                mjr_overlay(mjFONT_NORMAL,
                    mjGRID_TOPLEFT,
                    scene_rect,
                    "qhrr-05",
                    status,
                    &sim->platform_ui->mjr_context());
                mjr_overlay(mjFONT_NORMAL,
                    mjGRID_BOTTOMLEFT,
                    scene_rect,
                    "H: home  G: gravity  M: magnetic  SPACE: pause  R: reset  ESC: exit",
                    nullptr,
                    &sim->platform_ui->mjr_context());
            };

            std::thread physics([&] {
                sim->Load(model.get(), data.get(), options.model.c_str());
                auto next_tick = std::chrono::steady_clock::now();

                while (!sim->exitrequest.load() && !stopped) {
                    double tick_seconds = 0;

                    {
                        const mujoco::MutexLock lock(sim->mtx);

                        if (finished())
                            break;

                        const bool dropped = sim->droploadrequest.exchange(false);
                        const bool reloaded = sim->uiloadrequest.exchange(0) != 0;

                        if (dropped || reloaded) {
                            std::snprintf(sim->load_error,
                                sizeof(sim->load_error),
                                "Only the configured qhrr-05 model is supported; restart daemon to reload.");
                            std::cerr << "[simulate] " << sim->load_error << std::endl;
                        }

                        guarded([&] {
                            if (sim->run) {
                                step();
                                sim->AddToHistory();
                            } else {
                                applied_command = update_io();

                                if (virtual_motors)
                                    virtual_motors->Update(model.get(), data.get());

                                mj_forward(model.get(), data.get());
                                check_holding();
                            }
                        });

                        const double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
                        tick_seconds = model->opt.timestep * slowdown;
                    }

                    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(tick_seconds));
                    const auto now = std::chrono::steady_clock::now();

                    if (next_tick < now - std::chrono::milliseconds(100)) {
                        std::cerr
                            << "[simulate] physics is behind wall time; resynchronizing pacing\n";
                        next_tick = now;
                    }

                    std::this_thread::sleep_until(next_tick);
                }

                sim->exitrequest.store(1);
            });

            sim->RenderLoop();
            sim->exitrequest.store(1);
            physics.join();

            if (failed && !stopped)
                throw std::runtime_error("simulation stopped after IO/control failure");
        }

        sample();
        {
            shm_utils::Lock lock(shm);
            shm.simulation_finished = true;
            shm_utils::Notify(shm);
        }

        std::cout << "[simulate] stopped at t=" << data->time << std::endl;

        return 0;
    } catch (const std::exception& error) {
        if (stopped)
            return 0;

        std::cerr << "[simulate] ERROR: " << error.what() << '\n';

        return 1;
    }
}
