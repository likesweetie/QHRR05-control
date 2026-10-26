#include "motor_can/openrobot_protocol.hpp"
#include "shm_utils.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <linux/can.h>
#include <limits>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    pid_t child = -1;
    pid_t logger = -1;
    int peer = -1;
    const std::string suffix = std::to_string(::getpid());
    const std::string name = "/qhrr05-can-test-" + suffix;
    const std::string config = "/tmp/qhrr05-can-test-" + suffix + ".yaml";
    const std::string csv_path = "/tmp/qhrr05-can-test-" + suffix + ".csv";

    try {
        Require(argc == 3, "expected can_hal and shm_logger executables");
        const auto zero = openrobot::EncodePosition({}, {});
        Require(zero == openrobot::Payload{0xC0, 0x80, 0, 0x80, 0, 0, 0, 0x80},
            "MIT zero vector mismatch");
        const auto limits = openrobot::EncodePosition({true, 12.5, -45, 500, 5}, {});
        Require(limits == openrobot::Payload{0xC0, 255, 255, 0, 15, 255, 255, 128},
            "MIT boundary vector mismatch");

        for (double invalid : {13.0, std::numeric_limits<double>::quiet_NaN()}) {
            bool rejected = false;
            try {
                openrobot::EncodePosition({true, invalid, 0, 1, 1}, {});
            } catch (const std::exception&) {
                rejected = true;
            }
            Require(rejected, "invalid position accepted");
        }

        const openrobot::Payload feedback{0xC0, 26, 0, 0xFE, 0xA6, 0xFF, 0, 0xE0};
        const auto measured = openrobot::DecodeFeedback(feedback);
        Require(std::abs(measured.position + 3.14) < 1e-12 &&
                    std::abs(measured.velocity + 1.5707963267948966) < 1e-12 &&
                    measured.current == -8.25,
            "signed feedback units mismatch");
        const openrobot::Payload profile{0xC4, 45, 53, 0xA0, 5, 0x84, 3, 1};
        openrobot::CheckProfile(profile, {});

        peer = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        Require(peer >= 0, "peer socket failed");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = ::if_nametoindex("vcan0");
        Require(::bind(peer, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
            "peer bind failed");
        std::ofstream(config)
            << "shm_name: " << name
            << "\ninterface: vcan0\nfrequency_hz: 1000\nstatistics_hz: 10\nmotor_ids: [1, 2, 3]\nmit: {velocity_max_rad_s: 45, torque_max_nm: 53, kt_output_nm_a: 1.44, gear_ratio: 9, kd_scale: 1}\n";

        child = ::fork();
        Require(child >= 0, "fork failed");

        if (child == 0) {
            ::execl(argv[1], argv[1], config.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }

        std::array<unsigned, 3> controls{}, releases{};
        unsigned parameter_queries = 0;
        const auto service = [&](bool reply = true) {
            for (;;) {
                can_frame frame{};
                const auto size = ::read(peer, &frame, sizeof(frame));

                if (size < 0 && errno == EAGAIN)
                    break;

                Require(size == sizeof(frame) && frame.can_dlc == 8, "invalid CAN request");
                const int index = int(frame.can_id) - 0x141;
                Require(index >= 0 && index < 3, "wrong CAN SID");
                const auto opcode = frame.data[0];
                openrobot::Payload response{};

                if (opcode == 0xC4) {
                    ++parameter_queries;
                    response = profile;
                } else if (opcode == 0xC1) {
                    response = {0xC1, 1};
                } else if (opcode == 0xC2) {
                    ++releases[index];
                    response = {0xC2};
                } else if (opcode == 0xC0) {
                    ++controls[index];
                    Require(
                        frame.data[7] == 128, "torque feedforward was not fixed to logical zero");
                    const unsigned p = (unsigned(frame.data[1]) << 8) | frame.data[2];
                    const double position = double(p) / 65535 * 25 - 12.5;
                    Require(std::abs(position - (index + 1) * 0.1) < 0.0004,
                        "controller q target changed in transport");
                    const unsigned kp = ((frame.data[4] & 15) << 8) | frame.data[5];
                    Require(kp == 328, "Kp packing mismatch");
                    response = feedback;
                } else {
                    throw std::runtime_error("non-MIT command sent");
                }

                if (reply) {
                    std::copy(response.begin(), response.end(), frame.data);
                    Require(
                        ::write(peer, &frame, sizeof(frame)) == sizeof(frame), "peer write failed");
                }
            }
        };

        const auto until = [&](auto predicate) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);

            while (!predicate()) {
                service();
                Require(std::chrono::steady_clock::now() < deadline,
                    "CAN integration condition timed out");
                int status = 0;
                Require(::waitpid(child, &status, WNOHANG) == 0, "HAL exited during test");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };

        until([&] {
            return parameter_queries >= 3;
        });
        shm_utils::Mapping mapping(name);
        auto& shm = *mapping.get();
        logger = ::fork();
        Require(logger >= 0, "CAN logger fork failed");

        if (!logger) {
            ::execl(argv[2],
                argv[2],
                "--can",
                name.c_str(),
                csv_path.c_str(),
                static_cast<char*>(nullptr));
            _exit(127);
        }
        const auto snapshot = [&] {
            shm_utils::Lock lock(shm);
            return shm.can_states[shm.can_state_sequence % 2];
        };
        until([&] {
            const auto s = snapshot();
            return s.motors[0].profile_verified && s.motors[1].profile_verified &&
                   s.motors[2].profile_verified;
        });
        Require(snapshot().motors[0].rx_frames <= 2, "HAL received its own requests");

        const auto command = [&](int axis, const char* position) {
            const std::string index = std::to_string(axis);
            const pid_t publisher = ::fork();
            Require(publisher >= 0, "target publisher fork failed");

            if (!publisher) {
                ::execl(argv[1],
                    argv[1],
                    "--target",
                    name.c_str(),
                    index.c_str(),
                    position,
                    "0",
                    "40",
                    "0.3",
                    static_cast<char*>(nullptr));
                _exit(127);
            }

            int status = 0;
            Require(::waitpid(publisher, &status, 0) == publisher && WIFEXITED(status),
                "target publisher failed");
            return WEXITSTATUS(status);
        };

        Require(command(0, "NaN") == 1, "CLI accepted non-finite q target");
        Require(command(0, "0.1") == 0 && command(1, "0.2") == 0 && command(2, "0.3") == 0,
            "position target CLI failed");

        {
            shm_utils::Lock lock(shm);
            ShmMotorTargets target;
            target.sequence = shm.motor_target_sequence + 1;
            target.published_ns = shm_utils::MonotonicNs();

            for (int i = 0; i < 3; ++i)
                target.motors[i] = {true, (i + 1) * 0.1, 0, 40, 0.3};

            shm.motor_targets[target.sequence % 2] = target;
            shm.motor_target_sequence = target.sequence;
        }

        until([&] {
            return controls[0] > 20 && controls[1] > 20 && controls[2] > 20 &&
                   snapshot().motors[2].feedback_sequence > 10;
        });
        auto state = snapshot();
        Require(state.motors[0].feedback.current == -8.25 &&
                    state.motors[0].feedback.position == measured.position,
            "feedback not published through SHM");
        const auto old_cycles = state.cycles;
        const auto old_feedback = state.motors[0].feedback_sequence;
        const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);

        while (std::chrono::steady_clock::now() < quiet_until) {
            service(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        state = snapshot();
        Require(state.running && state.cycles > old_cycles + 20,
            "HAL waits for feedback or stops on response delay");
        Require(state.motors[0].feedback_sequence < old_feedback + 10,
            "cached feedback was republished as a new measurement");

        can_frame bad{};
        bad.can_id = 0x141;
        bad.can_dlc = 1;
        bad.data[0] = 0xC0;
        Require(::write(peer, &bad, sizeof(bad)) == sizeof(bad), "malformed test write failed");
        until([&] {
            return snapshot().motors[0].invalid_frames == 1;
        });

        {
            shm_utils::Lock lock(shm);
            auto target = shm.motor_targets[shm.motor_target_sequence % 2];
            target.sequence = shm.motor_target_sequence + 1;
            target.motors[0].enabled = false;
            shm.motor_targets[target.sequence % 2] = target;
            shm.motor_target_sequence = target.sequence;
        }

        until([&] {
            return releases[0] >= 2;
        });
        const auto stopped_controls = controls[0];
        const auto released_until =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(30);

        while (std::chrono::steady_clock::now() < released_until) {
            service();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        Require(controls[0] == stopped_controls && !snapshot().motors[0].enabled,
            "C0 resumed after Release");
        ::kill(child, SIGTERM);
        int status = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

        while (::waitpid(child, &status, WNOHANG) == 0) {
            service();
            Require(std::chrono::steady_clock::now() < deadline, "HAL ignored shutdown");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        child = -1;
        service();
        Require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "HAL shutdown failed");
        Require(releases[0] >= 3 && releases[1] >= 2 && releases[2] >= 2,
            "shutdown omitted MIT Release");
        Require(!snapshot().running, "final HAL status still running");
        int logger_status = 0;
        Require(::waitpid(logger, &logger_status, 0) == logger && WIFEXITED(logger_status) &&
                    WEXITSTATUS(logger_status) == 0,
            "CAN logger did not finish after HAL shutdown");
        logger = -1;
        std::ifstream csv(csv_path);
        const std::string contents((std::istreambuf_iterator<char>(csv)), {});
        Require(contents.find("observed_feedback_interval_us") != std::string::npos &&
                    contents.find("-8.25") != std::string::npos,
            "CAN CSV did not record feedback or timing");
        Require(::shm_open(name.c_str(), O_RDWR, 0) < 0 && errno == ENOENT,
            "HAL did not unlink owned SHM");
        std::cout
            << "PASS: real vcan, MIT position/PD/zero-FF, signed feedback, no self echo, latest-value hold, malformed frame, Release, shutdown/unlink\n";
    } catch (const std::exception& error) {
        if (logger > 0) {
            ::kill(logger, SIGKILL);
            ::waitpid(logger, nullptr, 0);
        }
        if (child > 0) {
            ::kill(child, SIGKILL);
            ::waitpid(child, nullptr, 0);
            ::shm_unlink(name.c_str());
        }

        std::cerr << error.what() << '\n';
        return 1;
    }

    if (peer >= 0)
        ::close(peer);

    std::filesystem::remove(config);
    std::filesystem::remove(csv_path);
    return 0;
}
