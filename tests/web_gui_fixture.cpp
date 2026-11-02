#include "ipc/shm_utils.hpp"
#include "interfaces/input/interface.hpp"
#include "simulation_ipc_fixture.hpp"
#include "robot_model/backend.hpp"

#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    try {
        if (argc != 2 && argc != 3)
            throw std::runtime_error("expected test SHM name and optional FK test config");

        const auto model = argc == 3 ? qhrr05::MakeMujocoBackend(argv[2]) : nullptr;
        const auto fk = model ? model->Evaluate({.1, .5, -1.3}, false).position
                              : std::array<double, 3>{};

        shm_utils::Mapping owner(argv[1], true);
        auto& shm = *owner.get();
        shm.can_enabled = true;
        shm.simulation_available = true;
        auto& motors = shm.motors[0];
        motors.running = true;
        motors.supported.fill(kCanSignals);
        motors.valid.fill(kCanSignals);
        motors.coordinate_valid.fill(true);
        motors.measurement_ns.fill(shm_utils::MonotonicNs());
        motors.feedback[0] = {.1, .2, 1.5, 27};
        motors.feedback[1] = {.5, -.3, -2, 28};
        motors.feedback[2] = {-1.3, .4, 3, 29};
        motors.motor_torque = {2.16, -2.88, 4.32};
        motors.profile_verified.fill(true);
        motors.enabled.fill(true);
        motors.received_sequence.fill(10);
        motors.received_ns.fill(shm_utils::MonotonicNs());
        shm.control[0].requested_drive_action = DriveAction::Arm;
        shm.control[0].drive_request_sequence = 1;
        shm.model[0].valid = true;
        shm.model[0].published_ns = shm_utils::MonotonicNs();
        shm.model[0].toe_position = {.2, -.1, .08};
        shm.model[0].joint_min.fill(-3);
        shm.model[0].joint_max.fill(3);
        shm.model[0].mass = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        shm.model[0].jacobian = {.1, 0, 0, 0, .1, 0, 0, 0, .1};
        shm.control[0].published_ns = shm_utils::MonotonicNs();
        shm.control[0].target_force = 3;
        shm.control[0].joystick_sequence = 37;
        SimulationData simulation;
        simulation.time = 12.5;
        simulation.published_ns = shm_utils::MonotonicNs();
        SimulationIpcFixture simulation_ipc(argv[1], simulation);
        std::cout << "ready" << std::endl;
        std::string command;

        while (std::getline(std::cin, command)) {
            if (command == "joystick-invalid" || command == "joystick-disconnect") {
                ControlData sample;

                if (command == "joystick-invalid") {
                    sample.joystick_connected = true;
                    sample.joystick_axes[0] = std::numeric_limits<double>::quiet_NaN();
                }

                qhrr05::InputInterface::PublishJoystick(shm, sample);
                std::cout << command << std::endl;
                continue;
            }

            if (command == "gui-homing" || command == "gui-invalid") {
                qhrr05::InputInterface::RequestMode(
                    shm, command == "gui-homing" ? 0 : 99, TrackingPlane::XY, 3);
                std::cout << command << std::endl;
                continue;
            }

            shm_utils::Lock lock(shm);

            if (command == "stop")
                shm.stopping = true;
            else if (command == "not-ready")
                shm.model[shm.model_sequence % kShmBufferCount].published_ns = 0;
            else if (command == "apply") {
                auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                control.mode = control.requested_mode;
                control.tracking_plane = control.requested_tracking_plane;
                control.active_motion_sequence = control.motion_sequence;
                ++shm.control_sequence;
            } else if (command.rfind("reaction", 0) == 0) {
                auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                auto& sample = shm_utils::Next(shm.motors, shm.motor_sequence);
                control.mode = 2;
                control.drive_state = DriveState::Active;
                control.magnetic.observer_valid = true;
                control.magnetic.phase = MagneticPhase::Following;
                control.magnetic.estimated_force = {-3, 4, 0};
                control.magnetic.normal = {1, 0, 0};
                control.magnetic.estimated_normal_force = 3;
                if (command == "reaction-invalid") control.magnetic.observer_valid = false;
                else if (command == "reaction-free") control.magnetic.phase = MagneticPhase::Free;
                else if (command == "reaction-inactive") control.mode = 5;
                else if (command == "reaction-disarmed") control.drive_state = DriveState::Disarmed;
                else if (command == "reaction-nan")
                    control.magnetic.estimated_force[0] = std::numeric_limits<double>::quiet_NaN();
                else if (command != "reaction") throw std::runtime_error("unknown reaction fixture instruction");
                control.published_ns = shm_utils::MonotonicNs();
                sample.received_ns.fill(control.published_ns);
                ++shm.control_sequence;
                ++shm.motor_sequence;
            } else if (command.rfind("tracking", 0) == 0 && model) {
                auto& control = shm_utils::Next(shm.control, shm.control_sequence);
                auto& sample = shm_utils::Next(shm.motors, shm.motor_sequence);
                if (command == "tracking" || command == "tracking-reset") {
                    control.mode = 3;
                    control.drive_state = DriveState::Active;
                    control.tracking_checking = false;
                    sample.coordinate_valid.fill(true);
                    ++control.active_motion_sequence;
                    const std::array<double, 3> error = command == "tracking"
                        ? std::array<double, 3>{.003, -.004, .012}
                        : std::array<double, 3>{.006, .008, 0};
                    for (int i = 0; i < 3; ++i)
                        control.tracking_position_reference[i] = fk[i] + error[i];
                } else if (command == "tracking-invalid") sample.coordinate_valid[0] = false;
                else if (command == "tracking-checking") control.tracking_checking = true;
                else if (command == "tracking-stop") control.mode = 5;
                else if (command != "tracking-sample")
                    throw std::runtime_error("unknown tracking fixture instruction");
                control.published_ns = shm_utils::MonotonicNs();
                sample.received_ns.fill(control.published_ns);
                sample.measurement_ns.fill(control.published_ns);
                for (auto& sequence : sample.received_sequence) ++sequence;
                // Deliberately wrong model/command values detect use of cached FK or MIT targets.
                shm.model[0].toe_position = {9, 9, 9};
                sample.reference.fill(2.8);
                for (auto& target : sample.command) target.position = 2.7;
                ++shm.control_sequence;
                ++shm.motor_sequence;
            } else throw std::runtime_error("unknown fixture instruction");

            std::cout << command << std::endl;
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
