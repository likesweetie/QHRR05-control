#include <array>
#include "simulation/simulation.hpp"
#include "control/limits.hpp"
#include "simulation/interaction.hpp"
#include "daemon/daemon.hpp"
#include "interfaces/input/interface.hpp"
#include "simulation/telemetry.hpp"
#include "simulation/workspace_overlay.hpp"
#include "simulate.h"
#include "glfw_adapter.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
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

    // A headless visualization session can run until the daemon requests shutdown.
    // Explicit --duration remains available for bounded tests.

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

        if (!data)
            throw std::runtime_error("could not allocate model data");

        qhrr05::ResetSimulation(initial, model.get(), data.get());

        {
            shm_utils::Lock lock(shm);

            if (shm.can_enabled == options.without_can)
                throw std::runtime_error("SHM transport does not match simulator mode");

            auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);

            for (int i = 0; i < 3; ++i)
                motors.position_offset[i] = options.without_can ? initial[i] : 0;

            ++shm.motor_sequence;
        }

        std::uint64_t run_id = 0;
        auto plots = std::make_unique<qhrr05::viewer::LivePlots>(history_sec);
        std::array<double, 3> reference = initial;
        std::uint64_t measurement_sequence = 0;

        std::unique_ptr<qhrr05::VirtualMotors> virtual_motors;

        if (!options.without_can) {
            virtual_motors = std::make_unique<qhrr05::VirtualMotors>(
                options.hal_config, options.simulation_config, initial, shm);
        }

        const auto configured_limits = qhrr05::ReadControlTorqueLimits(config);

        std::array<double, 3> torque_limits;

        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(configured_limits[i]) || configured_limits[i] <= 0 ||
                !model->actuator_ctrllimited[i] || model->actuator_ctrlrange[2 * i] >= 0 ||
                model->actuator_ctrlrange[2 * i + 1] <= 0)
                throw std::runtime_error("invalid controller/model torque limits for headroom");

            torque_limits[i] = std::min({configured_limits[i],
                -model->actuator_ctrlrange[2 * i],
                model->actuator_ctrlrange[2 * i + 1]});

            if (virtual_motors)
                torque_limits[i] = std::min(torque_limits[i], virtual_motors->torque_limit_nm());
        }

        const auto update_io = [&] {
            if (!virtual_motors)
                return qhrr05::UpdateControlIO(shm,
                    model.get(),
                    data.get(),
                    nullptr,
                    reference,
                    measurement_sequence,
                    nullptr,
                    false);

            const auto feedback = virtual_motors->ReadFeedback();

            if (!feedback.running) {
                stopped = 1;
                throw std::runtime_error("CAN HAL stopped normally");
            }

            // Control owns the measured-state model, for real and simulated motors alike.
            return qhrr05::UpdateControlIO(
                shm, model.get(), data.get(), nullptr, reference, 0, &feedback, false);
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

        qhrr05::InputInterface::RequestForce(shm, target_force);

        ControlData applied_command = update_io();

        std::uint64_t request_sequence = 0;
        int mode = 5;  // Startup/reset is pure damping. H explicitly starts homing.
        TrackingPlane tracking_plane = TrackingPlane::XY;
        double next_sample = 0;
        bool holding_reported = false;
        bool physics_paused = false;
        qhrr05::SimulationInteraction remote_interaction(options.shm_name, model.get(), data.get());

        const auto sync_requests = [&] {
            if (request_sequence != applied_command.motion_sequence)
                holding_reported = false;

            request_sequence = applied_command.motion_sequence;
            mode = applied_command.requested_mode;
            tracking_plane = applied_command.requested_tracking_plane;
            target_force = applied_command.target_force;
        };

        const auto request_control = [&](int requested) {
            return qhrr05::InputInterface::RequestMode(
                shm, requested, tracking_plane, target_force);
        };

        const auto reset = [&] {
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            ++run_id;
            plots->Clear();

            reference = initial;
            next_sample = 0;
            holding_reported = false;
            mode = 5;

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
                      << (mode == 0      ? "HOMING"
                             : mode == 1 ? "GRAVITY"
                             : mode == 2 ? "MAGNETIC"
                             : mode == 3 ? "TRACKING"
                             : mode == 4 ? "BALL FOLLOWING"
                                         : "DAMPING")
                      << " at t=" << data->time << std::endl;
        };

        const auto select_tracking_plane = [&](TrackingPlane plane) {
            if (mode == 3 && plane == tracking_plane)
                return;

            tracking_plane = plane;
            mode = 3;
            plots->Clear();
            request_sequence = request_control(mode);
            holding_reported = false;

            std::cout << "[control] TRACKING " << TrackingPlaneName(plane) << " at t=" << data->time
                      << std::endl;
        };

        SimulationData simulation_snapshot;
        const auto sample = [&] {
            qhrr05::viewer::Sample frame;
            frame.time = data->time;
            frame.run_id = run_id;
            frame.reference = reference;

            std::copy_n(data->mocap_pos + 3 * object_mocap, 3, frame.object_position.begin());
            const auto ground_truth = qhrr05::ReadContactGroundTruth(model.get(), data.get());
            std::copy_n(data->site_xpos + 3 * toe_site, 3, frame.toe_position.begin());
            std::copy_n(ground_truth.begin(), 3, frame.contact_force.begin());
            std::copy_n(ground_truth.begin() + 3, 3, frame.contact_torque.begin());

            std::copy_n(data->qpos, 3, frame.position.begin());

            auto& snapshot = simulation_snapshot;
            {
                shm_utils::Lock lock(shm);
                const auto& control = shm.control[shm.control_sequence % kShmBufferCount];
                const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];
                frame.mode = control.mode;
                frame.control_time = control.control_time;
                frame.magnetic = control.magnetic;
                frame.tracking_reference = control.tracking_position_reference;
                frame.tracking_plane = control.tracking_plane;
                frame.motion_sequence = control.active_motion_sequence;
                frame.tracking_method = control.tracking_method;
                const bool ctc = control.tracking_method == TrackingMethod::CTC;
                frame.dynamics_enabled = ctc ? control.mode == 3 && !control.tracking_ik_failed &&
                                                   !control.tracking_checking
                                             : control.tracking_idff_enabled;
                frame.dynamics_torque =
                    ctc ? control.tracking_ctc_torque : control.tracking_inverse_dynamics_torque;
                frame.commanded_torque = motors.debug_torque;
                frame.measured_torque = motors.motor_torque;
                frame.ball_position = control.ball_position;
                frame.ball_reference = control.ball_toe_reference;
            }
            snapshot.run_id = frame.run_id;
            snapshot.time = frame.time;
            snapshot.position_gt = frame.position;
            std::copy_n(data->actuator_force, 3, snapshot.torque_gt.begin());
            snapshot.toe_position = frame.toe_position;
            snapshot.contact_force = frame.contact_force;
            snapshot.contact_torque = frame.contact_torque;
            snapshot.object_position = frame.object_position;
            std::copy_n(data->mocap_quat + 4 * object_mocap, 4, snapshot.object_quaternion.begin());
            std::copy_n(data->qvel, 3, snapshot.velocity_gt.begin());
            remote_interaction.Publish(snapshot, physics_paused);
            const bool contact = mju_norm3(frame.contact_force.data()) > 0;

            if (contact != snapshot.contact) {
                ++snapshot.contact_event_sequence;
                snapshot.contact_event_time = frame.time;
            }

            snapshot.contact = contact;
            snapshot.published_ns = shm_utils::MonotonicNs();
            remote_interaction.PublishState(snapshot);

            plots->Push(frame);
        };

        auto next_paused_sample = std::chrono::steady_clock::now();
        const auto sample_paused = [&] {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_paused_sample) {
                sample();
                next_paused_sample =
                    now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(1.0 / sample_hz));
            }
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
            sync_requests();

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
                const auto interaction = remote_interaction.Poll(physics_paused);
                physics_paused = interaction.paused;
                if (interaction.reset)
                    reset();
                remote_interaction.Apply(physics_paused);
                if (!physics_paused || interaction.step) {
                    step();
                } else {
                    applied_command = update_io();
                    sync_requests();
                    if (virtual_motors)
                        virtual_motors->Update(model.get(), data.get());
                    mj_forward(model.get(), data.get());
                    check_holding();
                    sample_paused();
                }

                if (virtual_motors || physics_paused) {
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
            qhrr05::viewer::WorkspaceOverlay workspace_overlay;

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

            sim->user_scene_callback_ = [&](mjvScene& scene) {
                guarded([&] {
                    plots->AddToeTrail(scene);
                    workspace_overlay.Draw(scene);

                    if (applied_command.mode == 4) {
                        if (scene.ngeom == scene.maxgeom)
                            throw std::runtime_error(
                                "MuJoCo scene has no room for the target ball");

                        // Slightly larger than the foot so the initial target remains visible.
                        const double size[] = {.025, .025, .025};
                        const float rgba[] = {.25f, .75f, 1.f, .35f};
                        auto& ball = scene.geoms[scene.ngeom++];
                        mjv_initGeom(&ball,
                            mjGEOM_SPHERE,
                            size,
                            applied_command.ball_position.data(),
                            nullptr,
                            rgba);
                        ball.category = mjCAT_DECOR;
                        ball.objtype = mjOBJ_UNKNOWN;
                        ball.objid = ball.segid = -1;
                        ball.emission = .3f;
                    }
                });
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

                    qhrr05::InputInterface::RequestForce(shm, target_force);
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

                if (key == 'D' || key == 'd') {
                    sim->user_controller_callback_(5);
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

                if (key == 'T' || key == 't') {
                    sim->user_controller_callback_(3);
                    return true;
                }

                if (key == 'B' || key == 'b') {
                    sim->user_controller_callback_(4);
                    return true;
                }

                if ((key == 'W' || key == 'w') && !sim->ui0.editsect && !sim->ui1.editsect &&
                    !sim->uistate.control && !sim->uistate.alt) {
                    guarded([&] {
                        workspace_overlay.Toggle(options.config);
                    });
                    return true;
                }

                if (key >= '1' && key <= '3' && !sim->ui0.editsect && !sim->ui1.editsect &&
                    !sim->uistate.control && !sim->uistate.alt) {
                    guarded([&] {
                        select_tracking_plane(static_cast<TrackingPlane>(key - '1'));
                    });
                    return true;
                }

                return false;
            };

            sim->user_render_callback_ = [&] {
                const mujoco::MutexLock lock(sim->mtx);
                sim->user_target_force_ = target_force;

                if (stopped)
                    sim->exitrequest.store(1);

                const auto rect = sim->uistate.rect[3];
                const int width = rect.width * sim->user_panel_percent_ / 100;
                plots->Draw({rect.left + rect.width - width, rect.bottom, width, rect.height},
                    &sim->platform_ui->mjr_context(),
                    torque_limits);

                double max_error = 0;

                for (int i = 0; i < 3; ++i)
                    max_error = std::max(max_error, std::abs(home[i] - data->qpos[i]));

                char status[2048];
                std::snprintf(status,
                    sizeof(status),
                    "%s%s | sim=%.2f s ctrl=%.2f s | home error=%.4f rad",
                    mode == 2 ? MagneticPhaseName(applied_command.magnetic.phase)
                              : (mode == 1 ? "GRAVITY" : (holding_reported ? "HOLD" : "HOMING")),
                    !sim->run ? " / PAUSED" : "",
                    data->time,
                    applied_command.control_time,
                    max_error);

                if (mode == 5) {
                    const auto& target = applied_command.damping_toe_reference;

                    if (applied_command.mode == 5 && applied_command.damping_reference_valid)
                        std::snprintf(status,
                            sizeof(status),
                            "DAMPING%s | kp=0 kd=1 | no position stiffness\n"
                            "Toe target: %.3f / %.3f / %.3f m\nsim=%.2f ctrl=%.2f",
                            !sim->run ? " / PAUSED" : "",
                            target[0],
                            target[1],
                            target[2],
                            data->time,
                            applied_command.control_time);
                    else
                        std::snprintf(status,
                            sizeof(status),
                            "DAMPING | kp=0 kd=1 | awaiting controller/model sample");
                } else if (mode == 4) {
                    double error_squared = 0;

                    for (int i = 0; i < 3; ++i)
                        error_squared += std::pow(
                            applied_command.ball_position[i] - data->site_xpos[3 * toe_site + i],
                            2);

                    std::snprintf(status,
                        sizeof(status),
                        "BALL FOLLOWING%s | %s\nBall error=%.1f mm | ball speed=%.2f m/s\n%s | torque=%s",
                        !sim->run ? " / PAUSED" : "",
                        applied_command.joystick_connected ? "JOYSTICK CONNECTED" : "NO JOYSTICK",
                        1000 * std::sqrt(error_squared),
                        applied_command.ball_speed_m_s,
                        applied_command.ball_entering_workspace ? "ENTERING SAFE WORKSPACE"
                        : applied_command.ball_workspace_limited
                            ? "TARGET PROJECTED TO FK WORKSPACE"
                            : "FOLLOWING",
                        applied_command.ball_torque_saturated ? "SATURATED" : "OK");
                } else if (mode == 3) {
                    double error_squared = 0;
                    bool reference_outside_limits = false;
                    const bool ctc = applied_command.tracking_method == TrackingMethod::CTC;
                    const bool impedance =
                        applied_command.tracking_method == TrackingMethod::Impedance;
                    const bool controller_enabled =
                        applied_command.mode == 3 && !applied_command.tracking_ik_failed &&
                        !applied_command.tracking_checking &&
                        (ctc || impedance || applied_command.tracking_idff_enabled);

                    for (int i = 0; i < 3; ++i) {
                        const double error = applied_command.tracking_position_reference[i] -
                                             data->site_xpos[3 * toe_site + i];
                        error_squared += error * error;
                        reference_outside_limits |= reference[i] < model->jnt_range[2 * i] ||
                                                    reference[i] > model->jnt_range[2 * i + 1];
                    }

                    std::snprintf(status,
                        sizeof(status),
                        "%s %s%s | sim=%.2f ctrl=%.2f\nToe error=%.2f mm | torque=%s\n%s=%s | IK error=%.3f mm | q_d=%s | check=%.0f%%",
                        applied_command.tracking_checking    ? "CHECKING TRAJECTORY - HOLD"
                        : applied_command.tracking_ik_failed ? "IK / CONDITION BLOCKED - HOLD"
                                                             : "TRACKING",
                        TrackingPlaneName(applied_command.tracking_plane),
                        !sim->run ? " / PAUSED" : "",
                        data->time,
                        applied_command.control_time,
                        1000 * std::sqrt(error_squared),
                        applied_command.tracking_torque_saturated ? "SATURATED" : "OK",
                        ctc         ? "CTC"
                        : impedance ? "IMPEDANCE"
                                    : "IDFF",
                        controller_enabled ? "ON" : "OFF",
                        1000 * applied_command.tracking_ik_error_m,
                        reference_outside_limits ? "OUTSIDE LIMITS" : "OK",
                        100 * applied_command.tracking_check_progress);

                    const auto used = std::strlen(status);

                    if (applied_command.tracking_ilc_state == TrackingILCState::Off) {
                        std::snprintf(status + used, sizeof(status) - used, "\nILC: OFF");
                    } else {
                        char rmse[32];

                        if (std::isfinite(applied_command.tracking_ilc_last_rmse_m))
                            std::snprintf(rmse,
                                sizeof(rmse),
                                "%.2f mm",
                                1000 * applied_command.tracking_ilc_last_rmse_m);
                        else
                            std::snprintf(rmse, sizeof(rmse), "N/A");

                        const auto& ilc_torque = applied_command.tracking_ilc_torque;
                        std::snprintf(status + used,
                            sizeof(status) - used,
                            "\nILC: %s | updates=%llu skip=%llu | lap RMS=%s"
                            "\nILC HR/HP/KP: %+.2f / %+.2f / %+.2f Nm%s",
                            TrackingILCStateName(applied_command.tracking_ilc_state),
                            static_cast<unsigned long long>(applied_command.tracking_ilc_updates),
                            static_cast<unsigned long long>(
                                applied_command.tracking_ilc_skipped_cycles),
                            rmse,
                            ilc_torque[0],
                            ilc_torque[1],
                            ilc_torque[2],
                            applied_command.tracking_ilc_limited ? " [ILC LIMIT]" : "");
                    }
                } else if (mode == 2) {
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

                if (mode == 3) {
                    const auto headroom = plots->TorqueHeadroom(torque_limits, sample_hz);
                    const auto used = std::strlen(status);
                    std::snprintf(status + used, sizeof(status) - used, "%s", headroom.c_str());
                }

                const mjrRect scene_rect{rect.left, rect.bottom, rect.width - width, rect.height};
                const auto used = std::strlen(status);
                std::snprintf(status + used,
                    sizeof(status) - used,
                    "\nWorkspace [W]: %s",
                    workspace_overlay.Status().c_str());
                mjr_overlay(mjFONT_NORMAL,
                    mjGRID_TOPLEFT,
                    scene_rect,
                    "qhrr-05",
                    status,
                    &sim->platform_ui->mjr_context());
                mjr_overlay(mjFONT_NORMAL,
                    mjGRID_BOTTOMLEFT,
                    scene_rect,
                    "D: damping  H: home  G: gravity  M: magnetic  T: tracking  B: ball\n1: XY  2: XZ  3: YZ  W: workspace  L2: slow ball  R2: fast ball\nSPACE: pause  R: reset to damping  ESC: exit",
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

                        qhrr05::SimulationInteraction::Update interaction;
                        guarded([&] {
                            interaction = remote_interaction.Poll(!sim->run);
                            sim->run = !interaction.paused;
                            physics_paused = interaction.paused;
                            if (interaction.reset)
                                reset();
                            remote_interaction.Apply(physics_paused);
                        });
                        if (failed)
                            break;

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
                            if (sim->run || interaction.step) {
                                step();
                                sim->AddToHistory();
                            } else {
                                applied_command = update_io();
                                sync_requests();

                                if (virtual_motors)
                                    virtual_motors->Update(model.get(), data.get());

                                mj_forward(model.get(), data.get());
                                check_holding();
                                sample_paused();
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
        remote_interaction.Finish();

        std::cout << "[simulate] stopped at t=" << data->time << std::endl;

        return 0;
    } catch (const std::exception& error) {
        if (stopped)
            return 0;

        std::cerr << "[simulate] ERROR: " << error.what() << '\n';

        return 1;
    }
}
