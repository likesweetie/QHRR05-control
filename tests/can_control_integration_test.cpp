#include "mujoco_simulate/simulation.hpp"
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
        Require(argc == 6, "expected control/model/motor-config/HAL/controller paths");
        const std::string suffix = std::to_string(::getpid());
        directory = "/tmp/qhrr05-can-control-" + suffix;
        std::filesystem::create_directory(directory);
        const auto control_path = (directory / "control.yaml").string();
        const auto motor_path = (directory / "motor.yaml").string();
        const std::string sim_name = "/qhrr05-can-control-" + suffix;
        const std::string can_name = sim_name + "-bus";
        auto config = YAML::LoadFile(argv[1]);
        config["homing"]["duration_sec"] = .35;
        std::ofstream(control_path) << config;
        auto motor_config = YAML::LoadFile(argv[3]);
        motor_config["shm_name"] = can_name;
        std::ofstream(motor_path) << motor_config;

        const openrobot::Payload bytes{0xC0, 26, 0, 0xFE, 0xA6, 0xFF, 0, 0xE0};
        Require(openrobot::EncodeFeedback(openrobot::DecodeFeedback(bytes)) == bytes,
            "signed encoder feedback encoding mismatch");
        const auto decoded = openrobot::DecodePosition({0xC0, 255, 255, 0, 15, 255, 255, 128}, {});
        Require(decoded.position == 12.5 && decoded.velocity == -45 && decoded.kp == 500 &&
                    decoded.kd == 5,
            "MIT position decoding mismatch");

        hal = ::fork();
        Require(hal >= 0, "HAL fork failed");

        if (hal == 0) {
            ::execl(argv[4], argv[4], motor_path.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        std::unique_ptr<shm_utils::Mapping> can;

        while (!can) {
            if (std::filesystem::exists("/dev/shm/" + can_name.substr(1))) {
                can = std::make_unique<shm_utils::Mapping>(can_name);
                break;
            }

            Require(std::chrono::steady_clock::now() < deadline, "HAL did not create SHM");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        const auto snapshot = [&] {
            auto& shared = *can->get();
            shm_utils::Lock lock(shared);
            return shared.can_states[shared.can_state_sequence % 2];
        };

        while (!snapshot().running) {
            Require(std::chrono::steady_clock::now() < deadline, "HAL did not initialize");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        shm_utils::Mapping simulation(sim_name, true);
        auto& shm = *simulation.get();
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
        qhrr05::VirtualMotors motors(motor_path, origin);
        std::array<double, 3> reference{};

        const auto publish_model = [&] {
            const auto feedback = snapshot();

            for (int i = 0; i < 3; ++i) {
                sensed->qpos[i] = origin[i] + feedback.motors[i].feedback.position;
                sensed->qvel[i] = feedback.motors[i].feedback.velocity;
            }

            qhrr05::UpdateControlIO(
                shm, model.get(), sensed.get(), gravity.get(), reference, {}, 0, &feedback);
            shm_utils::Lock lock(shm);
            auto& state = shm.states[shm.state_sequence % 2];
            // Poison the legacy joint feedback: CAN mode must read encoders/current from HAL.
            state.position.fill(std::numeric_limits<double>::quiet_NaN());
            state.velocity = state.motor_torque = state.acceleration = state.position;
            shm.commands[shm.command_sequence % 2].torque.fill(1e6);
        };

        publish_model();
        controller = ::fork();
        Require(controller >= 0, "controller fork failed");

        if (controller == 0) {
            ::execl(argv[5],
                argv[5],
                control_path.c_str(),
                sim_name.c_str(),
                "--motor-config",
                motor_path.c_str(),
                static_cast<char*>(nullptr));
            _exit(127);
        }

        const auto request = [&](int mode) {
            shm_utils::Lock lock(shm);
            const auto sequence = shm.request_sequence + 1;
            shm.requests[sequence % 2] = {sequence, mode, 3, sequence};
            shm.request_sequence = sequence;
        };
        const auto command = [&] {
            shm_utils::Lock lock(shm);
            return shm.commands[shm.command_sequence % 2];
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

        for (int step = 0; step < 1500; ++step)
            tick();

        Require(command().holding, "CAN homing never reached HOLD");

        for (int i = 0; i < 3; ++i) {
            const auto state = snapshot();
            Require(state.motors[i].enabled && state.motors[i].feedback_sequence > 200,
                "MIT feedback did not traverse CAN");
            Require(std::abs(physical->qpos[i] - home[i]) < .08, "CAN homing did not track target");
            Require(
                std::abs(origin[i] + state.motors[i].feedback.position - physical->qpos[i]) < .01,
                "CAN position feedback differs from physics");
        }

        std::cout << "PASS: real CAN homing with poisoned legacy joint feedback/torque\n";
        ::kill(hal, SIGSTOP);
        mju_zero(physical->qvel, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        motors.Update(model.get(), physical.get());
        const std::array<double, 3> held{physical->ctrl[0], physical->ctrl[1], physical->ctrl[2]};
        const auto previous_sequence = command().sequence;
        request(1);

        for (int step = 0; step < 60; ++step) {
            tick(false);

            for (int i = 0; i < 3; ++i)
                Require(physical->ctrl[i] == held[i], "SHM command bypassed stopped CAN");
        }

        Require(command().mode == 1 && command().sequence > previous_sequence + 10,
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
            Require(!snapshot().motors[i].enabled && physical->ctrl[i] == 0,
                "controller exit did not release virtual motor");

        stop(hal);
        Require(!std::filesystem::exists("/dev/shm/" + can_name.substr(1)), "HAL left owned SHM");
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
