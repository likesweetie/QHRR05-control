#include "ipc/shm_utils.hpp"
#include "interfaces/input/interface.hpp"
#include "simulation_ipc_fixture.hpp"

#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    try {
        if (argc != 2)
            throw std::runtime_error("expected test SHM name");

        shm_utils::Mapping owner(argv[1], true);
        auto& shm = *owner.get();
        shm.can_enabled = true;
        auto& motors = shm.motors[0];
        motors.running = true;
        motors.position_offset = {0, .3, -1};
        motors.feedback[0] = {.1, .2, 1.5, 27};
        motors.feedback[1] = {.2, -.3, -2, 28};
        motors.feedback[2] = {-.3, .4, 3, 29};
        motors.motor_torque = {2.16, -2.88, 4.32};
        motors.profile_verified.fill(true);
        motors.enabled.fill(true);
        motors.received_sequence.fill(10);
        motors.received_ns.fill(shm_utils::MonotonicNs());
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
            } else
                throw std::runtime_error("unknown fixture instruction");

            std::cout << command << std::endl;
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
