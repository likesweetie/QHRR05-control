#include "simulation/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <linux/can.h>
#include <memory>
#include <net/if.h>
#include <sys/socket.h>
#include <thread>

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const char* message) {
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
}  // namespace

int main(int argc, char** argv) {
    int peer = -1;
    std::filesystem::path directory;

    try {
        Require(argc == 4, "expected scene, HAL config and simulation config");
        directory = "/tmp/qhrr05-mit-motor-" + std::to_string(::getpid());
        std::filesystem::create_directory(directory);
        auto hal = YAML::LoadFile(argv[2]);
        auto profile = YAML::LoadFile(
            (std::filesystem::path(argv[2]).parent_path() / hal["motor_config"].as<std::string>())
                .string());
        profile["mit"]["kd_scale"] = 3;
        hal["motor_config"] = (directory / "motor.yaml").string();
        const auto hal_path = (directory / "hal.yaml").string();
        std::ofstream(directory / "motor.yaml") << profile;
        std::ofstream(hal_path) << hal;
        const auto mit = profile["mit"];
        const double velocity_max = mit["velocity_max_rad_s"].as<double>();
        const double torque_max = mit["torque_max_nm"].as<double>();
        const auto expected_profile = openrobot::EncodeProfile(velocity_max,
            torque_max,
            mit["kt_output_nm_a"].as<double>(),
            mit["gear_ratio"].as<double>(),
            3);
        const auto ids = hal["motor_ids"].as<std::vector<unsigned>>();

        shm_utils::Mapping owner("/qhrr05-mit-motor-" + std::to_string(::getpid()), true);
        auto& shm = *owner.get();
        shm.can_enabled = true;
        shm.motors[0].running = true;

        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        Require(bool(data), "could not allocate MuJoCo data");
        const std::array<double, 3> origin{0, .3, -1};
        qhrr05::ResetSimulation(origin, model.get(), data.get());
        auto motors = std::make_unique<qhrr05::VirtualMotors>(hal_path, argv[3], origin, shm);
        peer = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        Require(peer >= 0, "test CAN socket failed");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = ::if_nametoindex(hal["interface"].as<std::string>().c_str());
        Require(::bind(peer, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
            "test CAN bind failed");

        const auto exchange = [&](int axis, const openrobot::Payload& bytes) {
            can_frame frame{};
            frame.can_id = 0x140 + ids[axis];
            frame.can_dlc = 8;
            std::copy(bytes.begin(), bytes.end(), frame.data);
            Require(::write(peer, &frame, sizeof(frame)) == sizeof(frame), "test CAN send failed");
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

            for (;;) {
                motors->Update(model.get(), data.get());
                can_frame reply{};
                const auto count = ::read(peer, &reply, sizeof(reply));

                if (count == sizeof(reply)) {
                    Require(reply.can_id == frame.can_id && reply.can_dlc == 8,
                        "virtual motor replied with wrong SID/DLC");
                    openrobot::Payload result{};
                    std::copy_n(reply.data, 8, result.begin());
                    return result;
                }

                Require(count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
                    "test CAN receive failed");
                Require(std::chrono::steady_clock::now() < deadline, "virtual motor reply timeout");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };
        const auto command = openrobot::EncodeMitCommand({.2, -.3, 40, 8, 0}, velocity_max, 3);
        const auto target = openrobot::DecodeMitCommand(command, velocity_max, 3);
        const auto feedback = [&](int axis) {
            return openrobot::DecodeFeedback(exchange(axis, command));
        };
        const auto zero = [&](int axis, int centidegrees) {
            const auto raw = static_cast<std::uint16_t>(centidegrees);
            const openrobot::Payload request{0xC3,
                0,
                0,
                0,
                0,
                0,
                static_cast<std::uint8_t>(raw),
                static_cast<std::uint8_t>(raw >> 8)};
            Require(exchange(axis, request) == request, "C3 signed offset echo mismatch");
        };
        const auto advance = [&](double seconds) {
            data->time += seconds;
            motors->Update(model.get(), data.get());
        };
        const auto expected_torque = [&](int axis, double position) {
            return std::clamp(target.kp * (target.position - position) +
                                  target.kd * (target.velocity - data->qvel[axis]),
                std::max(-torque_max, model->actuator_ctrlrange[2 * axis]),
                std::min(torque_max, model->actuator_ctrlrange[2 * axis + 1]));
        };

        Require(exchange(0, {0xC4}) == expected_profile, "C4 profile mismatch");

        for (int axis = 0; axis < 3; ++axis) {
            Near(feedback(axis).position,
                data->qpos[axis],
                .0004,
                "known model zero did not return the model joint angle");
            Near(data->ctrl[axis], 0, 0, "C0 enabled a released motor");
        }

        // Mechanical motion before the first enable must not move a calibrated zero.
        data->qpos[0] = .14;
        Require(exchange(0, {0xC1}) == openrobot::Payload{0xC1, 3}, "C1 Kd scale echo mismatch");
        Near(data->ctrl[0], 0, 0, "C1 replayed a command received while disabled");
        Near(feedback(0).position, .14, .0004, "first C1 rebased the calibrated model zero");
        Near(data->ctrl[0], expected_torque(0, .14), 1e-10, "MIT PD output mismatch");
        data->qpos[0] += .03;
        data->qvel[0] = .4;
        advance(.005);
        Near(data->ctrl[0],
            expected_torque(0, .17),
            1e-10,
            "motor did not hold the last received target");
        exchange(0, {0xC1});
        Near(data->ctrl[0],
            expected_torque(0, .17),
            1e-10,
            "repeated C1 changed an active target/origin");

        // C3 changes the coordinate only and drops both active and queued targets.
        const double physical_position = data->qpos[0];
        zero(0, 3000);
        Near(data->qpos[0], physical_position, 0, "C3 moved the physical joint");
        Near(data->ctrl[0], 0, 0, "C3 did not clear active torque");
        Near(feedback(0).position,
            3.141592653589793 / 6,
            .0004,
            "C3 +30 degree offset is not in output-shaft coordinates");
        Near(data->ctrl[0], 0, 0, "C0 bypassed the zero hold");
        exchange(0, {0xC1});
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        feedback(0);
        Near(data->ctrl[0], 0, 0, "wall time consumed the paused physics zero hold");
        advance(.019);
        feedback(0);
        Near(data->ctrl[0], 0, 0, "zero hold ended before 20ms");
        advance(.001);
        Near(data->ctrl[0], 0, 0, "an ignored C0 was replayed when zero hold expired");
        Near(feedback(0).position,
            3.141592653589793 / 6,
            .0004,
            "C1 overwrote user zero during hold");
        Near(data->ctrl[0],
            expected_torque(0, 3.141592653589793 / 6),
            1e-10,
            "fresh C0 did not resume control at 20ms");

        Require(exchange(0, {0xC2}) == openrobot::Payload{0xC2}, "C2 ACK contains stale bytes");
        Near(data->ctrl[0], 0, 0, "C2 did not immediately release torque");
        data->qpos[0] += .04;
        feedback(0);
        Near(data->ctrl[0], 0, 0, "C0 resumed a released motor");
        exchange(0, {0xC1});
        Near(data->ctrl[0], 0, 0, "C1 restored a stale target after release");
        Near(feedback(0).position,
            3.141592653589793 / 6 + .04,
            .0004,
            "re-enable did not preserve user zero");

        // Negative offset before enable, and each motor's independent hold/state.
        zero(1, -4500);
        advance(.020);
        Near(feedback(1).position,
            -3.141592653589793 / 4,
            .0004,
            "negative C3 offset before enable mismatch");
        Near(data->ctrl[1], 0, 0, "C3 enabled a released motor");
        exchange(1, {0xC1});
        Near(data->ctrl[1], 0, 0, "C1 replayed disabled target after C3");
        Near(feedback(1).position,
            -3.141592653589793 / 4,
            .0004,
            "C1 discarded a zero set before enable");
        zero(0, 0);
        Near(data->ctrl[1],
            expected_torque(1, -3.141592653589793 / 4),
            1e-10,
            "one motor's zero hold stopped another motor");
        Near(feedback(0).position, 0, .0004, "C3 zero did not clear prior output offset");
        advance(.010);
        zero(0, -32768);
        advance(.010);
        Near(feedback(0).position,
            -32768 * (3.141592653589793 / 18000),
            .0004,
            "C3 minimum signed int16 offset mismatch");
        Near(data->ctrl[0], 0, 0, "repeated C3 did not restart 20ms hold");
        exchange(0, {0xC2});
        advance(.010);
        feedback(0);
        Near(data->ctrl[0], 0, 0, "hold expiration enabled a released motor");

        zero(2, 0);
        exchange(2, {0xC1});
        feedback(2);
        Near(data->ctrl[2], 0, 0, "C1 from Released bypassed the outstanding zero hold");
        advance(.020);
        Near(data->ctrl[2], 0, 0, "C1 queued a target during zero hold");
        feedback(2);
        Near(data->ctrl[2],
            expected_torque(2, 0),
            1e-10,
            "C3-before-C1 sequence did not accept a fresh C0 after hold");

        // Rewinding physics preserves motor runtime zero; recreating motors clears it.
        exchange(0, {0xC1});
        zero(0, 32767);
        Near(feedback(0).position,
            32767 * (3.141592653589793 / 18000),
            .0004,
            "C3 maximum signed int16 offset mismatch");
        const double runtime_origin = data->qpos[0] - 32767 * (3.141592653589793 / 18000);
        qhrr05::ResetSimulation(origin, model.get(), data.get());
        Near(feedback(0).position,
            origin[0] - runtime_origin,
            .0004,
            "physics reset discarded runtime zero");
        Near(data->ctrl[0], 0, 0, "physics reset bypassed outstanding zero hold");
        advance(.020);
        Require(exchange(0, {0xC4}) == expected_profile, "C3 modified ROM parameters");
        motors.reset();
        motors = std::make_unique<qhrr05::VirtualMotors>(hal_path, argv[3], origin, shm);
        exchange(0, {0xC1});
        Near(feedback(0).position, origin[0], .0004, "motor power cycle retained runtime zero");
        exchange(1, {0xC1});
        Near(feedback(1).position,
            origin[1],
            .0004,
            "power cycle did not restore model joint coordinates");

        // Unknown encoder zeros are random only at construction, and remain fixed
        // across enable/release, pause and mechanical reset until an explicit C3.
        auto simulation = YAML::LoadFile(argv[3]);
        simulation["motor"]["require_zero_position_setup"] = true;
        const auto random_path = (directory / "random.yaml").string();
        std::ofstream(random_path) << simulation;
        motors.reset();
        motors = std::make_unique<qhrr05::VirtualMotors>(hal_path, random_path, origin, shm);
        std::array<double, 3> random_positions{};
        bool differs_from_model = false;

        for (int axis = 0; axis < 3; ++axis) {
            random_positions[axis] = feedback(axis).position;
            differs_from_model |= std::abs(random_positions[axis] - origin[axis]) > .01;
            exchange(axis, {0xC1});
            Near(feedback(axis).position,
                random_positions[axis],
                .0004,
                "enable overwrote the unknown encoder zero");
            exchange(axis, {0xC2});
            exchange(axis, {0xC1});
            Near(feedback(axis).position,
                random_positions[axis],
                .0004,
                "re-enable overwrote the unknown encoder zero");
        }

        Require(
            differs_from_model, "random zero policy still exposes calibrated model coordinates");
        data->qpos[0] += .03;
        Near(feedback(0).position,
            random_positions[0] + .03,
            .0004,
            "random encoder zero was resampled during motion");
        qhrr05::ResetSimulation(origin, model.get(), data.get());
        Near(feedback(0).position,
            random_positions[0],
            .0004,
            "mechanical reset changed the unknown encoder zero");
        zero(0, 0);
        advance(.020);
        exchange(0, {0xC1});
        Near(feedback(0).position, 0, .0004, "C3 did not calibrate an unknown encoder zero");

        ::close(peer);
        peer = -1;
        std::filesystem::remove_all(directory);
        std::cout
            << "PASS: real CAN MIT enable/release, model/random zero policies, signed C3 offset, "
               "20ms physics hold, stale command rejection, per-axis state, pause/reset\n";
        return 0;
    } catch (const std::exception& error) {
        if (peer >= 0)
            ::close(peer);

        if (!directory.empty())
            std::filesystem::remove_all(directory);

        std::cerr << error.what() << '\n';
        return 1;
    }
}
