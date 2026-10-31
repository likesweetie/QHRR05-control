#include "control/control.hpp"
#include "simulation/simulation.hpp"
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/wait.h>
#include <thread>

namespace {
void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    pid_t hal = -1;
    std::unique_ptr<shm_utils::Mapping> mapping;
    const auto root = "/tmp/qhrr-zero-can-" + std::to_string(getpid());
    const auto stop = [&] {
        if (mapping)
            shm_utils::Stop(*mapping->get());
        if (hal > 0) {
            kill(hal, SIGTERM);
            waitpid(hal, nullptr, 0);
            hal = -1;
        }
    };
    try {
        Require(argc == 6 || (argc == 7 && std::string(argv[6]) == "--default-speed"),
            "expected control/model/HAL config/HAL executable/simulation config [--default-speed]");
        std::filesystem::create_directory(root);
        const auto name = "/qhrr-zero-can-" + std::to_string(getpid());
        auto config = YAML::LoadFile(argv[1]);
        config["model_path"] = std::filesystem::absolute(argv[2]).string();
        // Faster traversal in this bounded test; all detector/effort settings are real defaults.
        if (argc == 6)
            config["zero_set"]["speed_rad_s"] = YAML::Load("[0.7, 0.7, 0.7]");
        std::ofstream(root + "/control.yaml") << config;
        auto sim_config = YAML::LoadFile(argv[5]);
        sim_config["motor"]["require_zero_position_setup"] = true;
        std::ofstream(root + "/simulation.yaml") << sim_config;
        mapping = std::make_unique<shm_utils::Mapping>(name, true);
        auto& shm = *mapping->get();
        shm.can_enabled = true;
        hal = fork();
        Require(hal >= 0, "fork failed");
        if (hal == 0) {
            auto* log = freopen((root + "/hal.log").c_str(), "w", stdout);
            if (!log)
                _exit(126);
            execl(argv[4], argv[4], argv[3], name.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }
        bool ready = false;
        for (int i = 0; i < 2000; ++i) {
            {
                shm_utils::Lock lock(shm);
                ready = shm.motors[shm.motor_sequence % kShmBufferCount].running;
            }
            if (ready)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        Require(ready, "HAL did not start");
        char error[2048]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        const auto initial =
            qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        qhrr05::ResetSimulation(initial, model.get(), data.get());
        qhrr05::VirtualMotors motors(argv[3], root + "/simulation.yaml", initial, shm);
        qhrr05::simulation_ipc::Server simulation(name);
        SimulationData state;
        state.require_zero_position_setup = true;
        state.published_ns = shm_utils::MonotonicNs();
        simulation.Publish(state);
        qhrr05::TaskController task(root + "/control.yaml", name);
        std::array<bool, 3> calibrated{};
        auto tick = std::chrono::steady_clock::now();
        const auto deadline = tick + std::chrono::seconds(argc == 7 ? 90 : 30);
        bool complete = false, external_contact_checked = false, motion_rejection_checked = false;
        bool check_rejection = false;
        while (std::chrono::steady_clock::now() < deadline) {
            simulation.Poll();
            state.time = data->time;
            state.published_ns = shm_utils::MonotonicNs();
            const auto contact = qhrr05::ReadContactGroundTruth(model.get(), data.get());
            state.contact = std::hypot(contact[0], contact[1], contact[2]) > 1e-6;
            simulation.Publish(state);  // Intentionally no position_gt: detector cannot use truth.
            motors.Update(model.get(), data.get());
            Require(task.ReadInput(), "controller stopped");
            task.Control();
            task.WriteOutput();
            ControlData control;
            MotorData feedback;
            {
                shm_utils::Lock lock(shm);
                control = shm.control[shm.control_sequence % kShmBufferCount];
                feedback = shm.motors[shm.motor_sequence % kShmBufferCount];
            }
            const auto& z = control.zero_set;
            if (z.phase == ZeroSetPhase::Fault && !external_contact_checked) {
                Require(z.failure == ZeroSetFailure::ExternalContact, "unexpected initial fault");
                for (bool enabled : feedback.command_enabled)
                    Require(!enabled, "external contact did not release motors");
                Require(!z.calibrated[0], "external obstacle was used as encoder zero");
                // Explicit operator preparation/retry, exactly like the other free-space
                // CAN tests. Never move scene objects inside the production controller.
                const int body = mj_name2id(model.get(), mjOBJ_BODY, "touch_object");
                data->mocap_pos[3 * model->body_mocapid[body]] = 5;
                mj_forward(model.get(), data.get());
                qhrr05::InputInterface::RequestMode(shm, 6, TrackingPlane::XY, 3);
                external_contact_checked = true;
                continue;
            }
            if (z.phase == ZeroSetPhase::Fault)
                throw std::runtime_error(
                    std::string("zero-set fault: ") + ZeroSetFailureName(z.failure));
            if (check_rejection) {
                Require(control.mode == 6 && control.requested_mode == 6,
                    "uncalibrated normal motion was not rejected and removed");
                motion_rejection_checked = true;
                check_rejection = false;
            }
            if (external_contact_checked && !motion_rejection_checked && !check_rejection &&
                z.phase == ZeroSetPhase::Seeking) {
                qhrr05::InputInterface::RequestMode(shm, 0, TrackingPlane::XY, 3);
                check_rejection = true;
            }
            for (int i = 0; i < 3; ++i) {
                if (z.calibrated[i] && !calibrated[i]) {
                    if (i)
                        Require(calibrated[i - 1], "axis calibrated out of order");
                    const double expected = config["zero_set"]["stop_position_rad"][i].as<double>();
                    std::cout << "axis " << i << " contact truth=" << data->qpos[i]
                              << " expected=" << expected << '\n';
                    Require(std::abs(data->qpos[i] - expected) < .04,
                        "false contact away from mechanical stop");
                    calibrated[i] = true;
                }
                const double limit = config["zero_set"]["maximum_push_nm"][i].as<double>();
                Require(std::abs(feedback.debug_torque[i]) <= limit + 1e-9,
                    "calibration effort exceeded bound");
            }
            if (z.phase == ZeroSetPhase::Complete) {
                for (int i = 0; i < 3; ++i)
                    Require(std::abs(feedback.position_offset[i] + feedback.feedback[i].position -
                                     data->qpos[i]) < .04,
                        "calibrated encoder does not match model coordinates");
                complete = true;
                break;
            }
            mj_step(model.get(), data.get());
            mj_forward(model.get(), data.get());
            tick += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(tick);
        }
        Require(complete, "calibration did not finish within test deadline");
        Require(external_contact_checked && motion_rejection_checked,
            "obstacle and normal-mode calibration gates were not exercised");
        // Physical reset is not a power cycle; calibrated runtime offsets must survive.
        MotorData before;
        {
            shm_utils::Lock lock(shm);
            before = shm.motors[shm.motor_sequence % kShmBufferCount];
        }
        qhrr05::ResetSimulation(initial, model.get(), data.get());
        for (int i = 0; i < 40; ++i) {
            motors.Update(model.get(), data.get());
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto after = motors.ReadFeedback();
        Require(after.position_offset == before.position_offset, "reset cleared runtime zero");
        for (int i = 0; i < 3; ++i)
            Require(
                std::abs(after.position_offset[i] + after.feedback[i].position - initial[i]) < .04,
                "reset encoder lost calibration");
        stop();
        std::filesystem::remove_all(root);
        std::cout
            << "PASS: random encoder startup, real CAN torque contact, hip/thigh/calf offsets and reset persistence\n";
        return 0;
    } catch (const std::exception& e) {
        stop();
        std::cerr << e.what() << " (diagnostics: " << root << ")\n";
        return 1;
    }
}
