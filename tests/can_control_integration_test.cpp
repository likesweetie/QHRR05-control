#include "simulation/simulation.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sys/wait.h>
#include <thread>

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    pid_t hal = -1, controller = -1;
    std::unique_ptr<shm_utils::Mapping> can;
    std::filesystem::path directory;
    const auto stop = [](pid_t& child) {
        if (child <= 0)
            return;

        ::kill(child, SIGCONT);
        ::kill(child, SIGTERM);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        int status = 0;

        while (::waitpid(child, &status, WNOHANG) == 0) {
            if (std::chrono::steady_clock::now() > deadline)
                ::kill(child, SIGKILL);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        child = -1;
        return;
    };

    try {
        Require(
            argc == 7 || argc == 8, "expected control/model/HAL-config/HAL/controller/simulation-config paths [ctc|impedance]");
        const std::string method = argc == 8 ? argv[7] : "inverse_dynamics";
        Require(method == "inverse_dynamics" || method == "ctc" || method == "impedance",
            "unknown CAN test tracking method");
        const bool use_ilc = method != "inverse_dynamics";
        const auto tracking_method = method == "ctc" ? TrackingMethod::CTC
            : method == "impedance" ? TrackingMethod::Impedance : TrackingMethod::InverseDynamics;
        const std::string suffix = std::to_string(::getpid());
        directory = "/tmp/qhrr05-can-control-" + suffix;
        std::filesystem::create_directory(directory);
        const auto control_path = (directory / "control.yaml").string();
        const auto hal_path = (directory / "HAL settings.yaml").string();
        const std::string sim_name = "/qhrr05-can-control-" + suffix;
        const std::string can_name = sim_name + "-bus";
        auto config = YAML::LoadFile(argv[1]);
        config["tracking"]["mode"] = method;
        if (use_ilc) {
            // Exercise the independent ILC switch over real CAN while retaining
            // the inverse-dynamics case as the unchanged ILC-OFF baseline.
            config["tracking"]["ilc"]["enabled"] = true;
            config["tracking"]["ilc"]["warmup_cycles"] = 0;
            config["tracking"]["ilc"]["phase_bins"] = 32;
        }
        config["model_path"] = std::filesystem::absolute(argv[2]).string();
        config["homing"]["duration_sec"] = .35;
        std::ofstream(control_path) << config;
        auto hal_config = YAML::LoadFile(argv[3]);
        const auto motor_source = std::filesystem::path(argv[3]).parent_path() /
                                  hal_config["motor_config"].as<std::string>();
        std::ofstream(directory / "motor profile.yaml") << YAML::LoadFile(motor_source.string());
        hal_config["motor_config"] = "motor profile.yaml";
        std::ofstream(hal_path) << hal_config;

        const openrobot::Payload bytes{0xC0, 26, 0, 0xFE, 0xA6, 0xFF, 0, 0xE0};
        Require(openrobot::EncodeFeedback(openrobot::DecodeFeedback(bytes)) == bytes,
            "signed encoder feedback encoding mismatch");
        const auto decoded =
            openrobot::DecodeMitCommand({0xC0, 255, 255, 0, 15, 255, 255, 128}, 45, 1);
        Require(decoded.position == 12.5 && decoded.velocity == -45 && decoded.kp == 500 &&
                    decoded.kd == 5 && decoded.tau_ff == 0,
            "MIT position decoding mismatch");

        can = std::make_unique<shm_utils::Mapping>(can_name, true);
        can->get()->can_enabled = true;

        hal = ::fork();
        Require(hal >= 0, "HAL fork failed");

        if (hal == 0) {
            ::execl(
                argv[4], argv[4], hal_path.c_str(), can_name.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        const auto snapshot = [&] {
            auto& shared = *can->get();
            shm_utils::Lock lock(shared);
            return shared.motors[shared.motor_sequence % kShmBufferCount];
        };

        while (!snapshot().running) {
            Require(std::chrono::steady_clock::now() < deadline, "HAL did not initialize");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto& shm = *can->get();
        char error[2048]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> physical(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> sensed(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity(
            mj_makeData(model.get()), mj_deleteData);
        const auto origin =
            qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        qhrr05::ResetSimulation(origin, model.get(), physical.get());
        {
            shm_utils::Lock lock(shm);
            auto& next = shm_utils::Next(shm.motors, shm.motor_sequence);

            for (int i = 0; i < 3; ++i)
                next.position_offset[i] = origin[i];

            ++shm.motor_sequence;
        }
        qhrr05::VirtualMotors motors(hal_path, argv[6], origin, shm);
        std::array<double, 3> reference{};
        bool force_unreachable_model = false;

        const auto publish_model = [&] {
            const auto feedback = snapshot();

            for (int i = 0; i < 3; ++i) {
                sensed->qpos[i] = origin[i] + feedback.feedback[i].position;
                sensed->qvel[i] = feedback.feedback[i].velocity;
            }

            qhrr05::UpdateControlIO(
                shm, model.get(), sensed.get(), gravity.get(), reference, 0, &feedback);
            shm_utils::Lock lock(shm);
            // Poison physics truth and debug effort; neither may drive CAN control.
            auto& truth = shm.simulation[shm.simulation_sequence % kShmBufferCount];
            truth.position_gt.fill(std::numeric_limits<double>::quiet_NaN());
            truth.torque_gt = truth.position_gt;

            shm.motors[shm.motor_sequence % kShmBufferCount].debug_torque.fill(1e6);

            if (force_unreachable_model)
                shm.model[shm.model_sequence % kShmBufferCount].toe_position = {10, 10, 10};
        };

        publish_model();
        controller = ::fork();
        Require(controller >= 0, "controller fork failed");

        if (controller == 0) {
            ::execl(argv[5],
                argv[5],
                control_path.c_str(),
                can_name.c_str(),
                static_cast<char*>(nullptr));
            _exit(127);
        }

        const auto request = [&](int mode) {
            shm_utils::Lock lock(shm);
            auto& control = shm_utils::Next(shm.control, shm.control_sequence);
            control.requested_mode = mode;
            control.target_force = 3;
            ++control.motion_sequence;
            ++shm.control_sequence;
        };
        const auto command = [&] {
            shm_utils::Lock lock(shm);
            return shm.control[shm.control_sequence % kShmBufferCount];
        };
        auto next = std::chrono::steady_clock::now();
        const auto tick = [&](bool integrate = true) {
            publish_model();
            motors.Update(model.get(), physical.get());

            if (integrate) {
                mj_step(model.get(), physical.get());
                mj_forward(model.get(), physical.get());
            }

            Require(::kill(controller, 0) == 0, "controller exited during CAN control");
            next += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next);
        };

        for (int step = 0; step < 6000 && !command().command_sequence; ++step)
            tick(false);

        Require(command().mode == 5 && !command().holding && command().damping_reference_valid,
            "CAN controller automatically homed instead of starting in damping");
        const auto initial_toe = command().damping_toe_reference;

        for (int i = 0; i < 3; ++i) {
            const auto target = snapshot().command[i];
            Require(target.kp == 0 && target.kd == 1 && target.velocity == 0 && target.tau_ff == 0,
                "startup MIT command is not kp=0/kd=1/velocity=0/FF=0");
            physical->qpos[i] += .01;
            physical->qvel[i] = .2 * (i + 1);
        }

        mj_forward(model.get(), physical.get());

        for (int step = 0; step < 50; ++step)
            tick(false);

        Require(command().damping_toe_reference == initial_toe,
            "CAN damping changed its captured toe target");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(physical->ctrl[i] + physical->qvel[i]) < .03,
                "actual MIT CAN damping is not pure -kd*dq");

        qhrr05::ResetSimulation(origin, model.get(), physical.get());
        std::cout << "PASS: actual CAN startup kp=0/kd=1, captured toe target, no position/gravity kick\n";
        request(0);  // Homing now requires an explicit request.

        for (int step = 0; step < 6000 && !command().holding; ++step)
            tick();

        for (int step = 0; step < 500; ++step)
            tick();

        Require(command().holding, "CAN homing never reached HOLD");

        for (int i = 0; i < 3; ++i) {
            const auto state = snapshot();
            Require(state.enabled[i] && state.received_sequence[i] > 200,
                "MIT feedback did not traverse CAN");
            Require(std::abs(physical->qpos[i] - home[i]) < .08, "CAN homing did not track target");
            Require(std::abs(origin[i] + state.feedback[i].position - physical->qpos[i]) < .01,
                "CAN position feedback differs from physics");
        }

        std::cout << "PASS: real CAN homing with poisoned legacy joint feedback/torque\n";
        for (const auto plane : {TrackingPlane::XY, TrackingPlane::XZ, TrackingPlane::YZ}) {
            {
                shm_utils::Lock lock(shm);
                auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                control.requested_tracking_plane = plane;
                control.requested_mode = 3;
                ++control.motion_sequence;
                ++shm.control_sequence;
            }

            double tracking_error_squared = 0;
            bool observed_ilc_torque = false;
            bool observed_checking = false;
            int tracking_samples = 0;
            const int tracking_site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");

            for (int step = 0; step < 3000; ++step) {
                tick();
                const auto control = command();

                if (control.tracking_checking) {
                    observed_checking = true;
                    Require(!control.tracking_idff_enabled && !control.tracking_ik_failed &&
                                control.tracking_check_progress >= 0 && control.tracking_check_progress < 1 &&
                                control.tracking_inverse_dynamics_torque == std::array<double, 3>{} &&
                                control.tracking_ctc_torque == std::array<double, 3>{} &&
                                control.tracking_ilc_torque == std::array<double, 3>{},
                        "CAN preflight published active trajectory/feedforward instead of holding");
                }

                for (double value : control.tracking_ilc_torque)
                    observed_ilc_torque |= std::abs(value) > 1e-5;

                if (step > 1500) {
                    Require(control.mode == 3 && control.tracking_plane == plane,
                        "Tracking mode did not run over CAN");
                    Require(control.tracking_idff_enabled == (tracking_method == TrackingMethod::InverseDynamics) &&
                                control.tracking_method == tracking_method &&
                                !control.tracking_ik_failed && control.tracking_ik_error_m <= 1e-5,
                        "CAN Tracking did not publish the selected method and valid IK");

                    if (tracking_method == TrackingMethod::Impedance)
                        Require(control.tracking_inverse_dynamics_torque == std::array<double, 3>{} &&
                                    control.tracking_ctc_torque == std::array<double, 3>{},
                            "CAN impedance mode included dynamics feedforward torque");

                    for (int i = 0; i < 3; ++i) {
                        const double error = control.tracking_position_reference[i] -
                                             physical->site_xpos[3 * tracking_site + i];
                        tracking_error_squared += error * error;
                        Require(snapshot().command[i].tau_ff == 0,
                            "Tracking sent nonzero torque feedforward");
                    }

                    ++tracking_samples;
                }
            }

            const double tracking_rmse = std::sqrt(tracking_error_squared / tracking_samples);
            Require(observed_checking && !command().tracking_checking && command().tracking_check_progress == 1,
                "CAN tracking did not publish a complete preflight before running");
            if (use_ilc)
                Require(command().tracking_ilc_updates > 0 && observed_ilc_torque,
                    "ILC did not learn or publish a nonzero correction over actual CAN");
            std::cout << "CAN tracking " << method << " " << TrackingPlaneName(plane)
                      << " RMSE=" << 1000 * tracking_rmse << " mm\n";
            Require(tracking_rmse < .025, "CAN tracking did not follow the Cartesian trajectory");
        }

        request(0);

        for (int step = 0; step < 1000; ++step)
            tick();

        Require(command().holding, "Tracking could not return to homing");

        request(4);

        for (int step = 0; step < 100; ++step)
            tick();

        Require(command().mode == 4, "Ball mode did not start over CAN");
        const auto initial_ball = command().ball_position;
        const int ball_site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        const std::array<double, 3> ball_start_toe{
            physical->site_xpos[3 * ball_site], physical->site_xpos[3 * ball_site + 1],
            physical->site_xpos[3 * ball_site + 2]};
        {
            shm_utils::Lock lock(shm);
            auto& control = shm_utils::Next(shm.control, shm.control_sequence);
            control.requested_ball_position = {1.5, .14, .2};
            ++control.ball_target_sequence;
            ++shm.control_sequence;
        }

        for (int step = 0; step < 1000; ++step) {
            tick();
            const auto state = snapshot();

            for (int i = 0; i < 3; ++i)
                Require(state.command[i].tau_ff == 0 && std::isfinite(state.command[i].position),
                    "Ball mode bypassed MIT position commands");
        }

        Require(command().ball_workspace_limited && command().ball_position[0] == 1.5,
            "Ball outside target was lost/clamped rather than projected for the foot");
        double ball_motion_squared = 0;

        for (int i = 0; i < 3; ++i)
            ball_motion_squared += std::pow(physical->site_xpos[3 * ball_site + i] - ball_start_toe[i], 2);

        Require(std::sqrt(ball_motion_squared) > .01, "Ball mode did not move the physical foot over CAN");
        request(0);

        for (int step = 0; step < 1000; ++step)
            tick();

        request(4);

        for (int step = 0; step < 50; ++step)
            tick();

        Require(command().ball_position[0] < .6,
            "Ball mode reentry retained the old unreachable target");
        request(0);

        for (int step = 0; step < 1000; ++step)
            tick();

        std::cout << "PASS: Ball following MIT commands, outside target, reentry reset\n";

        force_unreachable_model = true;
        publish_model();
        request(3);

        for (int step = 0; step < 100; ++step)
            tick();

        Require(command().tracking_ik_failed && !command().tracking_idff_enabled,
            "runtime IK failure did not publish HOLD with IDFF disabled");
        if (use_ilc)
            Require(command().tracking_ilc_state == TrackingILCState::IKHold &&
                        command().tracking_ilc_torque == std::array<double, 3>{},
                "IK failure retained an active ILC correction");
        const auto stopped_reference = snapshot().reference;
        const auto hold_sequence = command().command_sequence;
        force_unreachable_model = false;

        for (int step = 0; step < 500; ++step)
            tick();

        Require(command().tracking_ik_failed && command().command_sequence > hold_sequence + 100 &&
                    snapshot().reference == stopped_reference,
            "IK hold terminated control, resumed tracking or changed its captured target");

        for (int i = 0; i < 3; ++i) {
            Require(snapshot().command[i].tau_ff == 0 && snapshot().enabled[i],
                "IK hold disabled the motor or used torque FF instead of holding");
            Require(std::abs(physical->qpos[i] - stopped_reference[i]) < .05,
                "actual CAN IK hold moved away from its captured pose");
        }

        request(0);

        for (int step = 0; step < 1000; ++step)
            tick();

        request(3);

        for (int step = 0; step < 100; ++step)
            tick();

        Require(command().mode == 3 && !command().tracking_ik_failed,
            "Tracking reentry did not clear IK hold");
        Require(command().tracking_ilc_updates == 0,
            "Tracking reentry reused ILC corrections from the failed trajectory");
        request(0);

        for (int step = 0; step < 1000; ++step)
            tick();

        std::cout << "PASS: real CAN IK failure -> latched measured-pose hold, running loop, restart\n";

        ::kill(hal, SIGSTOP);
        mju_zero(physical->qvel, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        motors.Update(model.get(), physical.get());
        const std::array<double, 3> held{physical->ctrl[0], physical->ctrl[1], physical->ctrl[2]};
        const auto previous_sequence = command().command_sequence;
        request(1);

        for (int step = 0; step < 60; ++step) {
            tick(false);

            for (int i = 0; i < 3; ++i)
                Require(physical->ctrl[i] == held[i], "SHM command bypassed stopped CAN");
        }

        Require(command().mode == 1 && command().command_sequence > previous_sequence + 10,
            "controller waited for CAN feedback instead of running independently");
        std::cout
            << "PASS: stopped CAN retains ZOH; controller continues; no SHM actuation bypass\n";
        ::kill(hal, SIGCONT);

        for (int step = 0; step < 200; ++step)
            tick();

        request(2);

        for (int step = 0; step < 300; ++step)
            tick();

        Require(command().mode == 2 && std::isfinite(command().magnetic.estimated_normal_force),
            "Magnetic mode did not run over CAN");
        std::cout << "PASS: Gravity and Magnetic position targets over CAN\n";

        const int object = mj_name2id(model.get(), mjOBJ_BODY, "touch_object");
        const int mocap = model->body_mocapid[object];
        const int toe = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        const double toe_x = physical->site_xpos[3 * toe];
        physical->mocap_pos[3 * mocap + 1] = physical->site_xpos[3 * toe + 1];
        physical->mocap_pos[3 * mocap + 2] = physical->site_xpos[3 * toe + 2];
        int following = 0, contact_samples = 0;
        double contact_sum = 0, force_error_squared = 0;

        for (int step = 0; step < 3600; ++step) {
            const double time = step * .001;
            double center = toe_x + .09;

            if (time >= .2 && time < .8)
                center -= (time - .2) * .1;
            else if (time >= .8 && time < 2.4)
                center = toe_x + .03 + .0015 * std::sin(6.283185307 * (time - .8));
            else if (time >= 2.4)
                center = toe_x + .03 + std::min(.12, (time - 2.4) * .12);

            physical->mocap_pos[3 * mocap] = center;
            tick();
            const auto telemetry = command().magnetic;

            if (telemetry.phase == MagneticPhase::Following) {
                ++following;
                const auto truth = qhrr05::ReadContactGroundTruth(model.get(), physical.get());
                double force = 0;

                for (int i = 0; i < 3; ++i)
                    force -= truth[i] * telemetry.normal[i];

                if (force > .5) {
                    ++contact_samples;
                    contact_sum += force;
                    force_error_squared += (force - 3) * (force - 3);
                }
            }
        }

        std::cout << "CAN contact: following=" << following << " actual_contact=" << contact_samples
                  << " mean_force=" << (contact_samples ? contact_sum / contact_samples : 0)
                  << " RMSE="
                  << (contact_samples ? std::sqrt(force_error_squared / contact_samples) : 0)
                  << " final_phase=" << MagneticPhaseName(command().magnetic.phase) << '\n';
        Require(following > 200 && contact_samples > 100,
            "CAN Magnetic mode did not follow real physical contact");
        Require(command().magnetic.phase == MagneticPhase::Free,
            "CAN Magnetic mode did not release after object removal");

        stop(controller);

        for (int step = 0; step < 40; ++step) {
            motors.Update(model.get(), physical.get());
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        for (int i = 0; i < 3; ++i)
            Require(!snapshot().enabled[i] && physical->ctrl[i] == 0,
                "controller exit did not release virtual motor");

        stop(hal);
        Require(std::filesystem::exists("/dev/shm/" + can_name.substr(1)),
            "HAL unlinked daemon-owned SHM");
        can.reset();
        Require(
            !std::filesystem::exists("/dev/shm/" + can_name.substr(1)), "owner left SHM behind");
        std::filesystem::remove_all(directory);
        std::cout << "PASS: release and CAN ownership cleanup\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        stop(controller);
        stop(hal);

        if (!directory.empty())
            std::filesystem::remove_all(directory);

        return 1;
    }
}
