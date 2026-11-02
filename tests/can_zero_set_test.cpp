#include "ipc/shm_utils.hpp"
#include "motor_can_protocol/openrobot_protocol.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>

namespace {
void Require(bool value, const char* reason) {
    if (!value)
        throw std::runtime_error(reason);
}
enum class ReplyMode { Normal, WrongEcho, NoEcho, NoFeedback };
}

int main(int argc, char** argv) {
    pid_t child = -1;
    int peer = -1;
    std::filesystem::path directory;
    try {
        Require(argc == 2, "expected HAL executable");
        const std::string name = "/qhrr-c3-test-" + std::to_string(getpid());
        directory = "/tmp/qhrr-c3-test-" + std::to_string(getpid());
        std::filesystem::create_directory(directory);
        const auto config = (directory / "hal.yaml").string();
        std::ofstream(config)
            << "interface: vcan0\nfrequency_hz: 1000\nstatistics_hz: 10\n"
               "bus_load_bitrate: 1000000\ncommand_timeout_sec: 0.1\n"
               "feedback_timeout_sec: 0.25\nenable_timeout_sec: 1\npoll_released: false\n"
               "motor_ids: [1, 2, 3]\nmotor_config: motor.yaml\n";
        std::ofstream(directory / "motor.yaml")
            << "mit: {velocity_max_rad_s: 45, torque_max_nm: 33, kt_output_nm_a: 1.298, gear_ratio: 9, kd_scale: 1}\n";
        shm_utils::Mapping owner(name, true);
        auto& shm = *owner.get();
        shm.can_enabled = true;
        peer = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        Require(peer >= 0, "CAN peer socket failed");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = if_nametoindex("vcan0");
        Require(::bind(peer, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
            "CAN peer bind failed");
        child = fork();
        Require(child >= 0, "fork failed");
        if (!child) {
            if (!freopen((directory / "hal.log").c_str(), "w", stdout) ||
                !freopen((directory / "hal.log").c_str(), "a", stderr))
                _exit(126);
            execl(argv[1], argv[1], config.c_str(), name.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }
        const auto snapshot = [&] {
            shm_utils::Lock lock(shm);
            return shm.motors[shm.motor_sequence % kShmBufferCount];
        };
        bool enabled = false, resume = false;
        std::uint64_t request = 0, clear = 0;
        double requested_position = 0, motor_position = .2;
        unsigned c3_count = 0, active_commands = 0;
        ReplyMode mode = ReplyMode::Normal;
        std::uint64_t c3_time = 0, echo_time = 0;
        openrobot::Payload c3{};
        const auto send = [&](canid_t id, const openrobot::Payload& payload) {
            can_frame reply{};
            reply.can_id = id;
            reply.can_dlc = 8;
            std::copy(payload.begin(), payload.end(), reply.data);
            Require(write(peer, &reply, sizeof(reply)) == sizeof(reply), "peer response failed");
        };
        const auto pump = [&] {
            {
                shm_utils::Lock lock(shm);
                auto& out = shm_utils::Next(shm.motors, shm.motor_sequence);
                out.command_enabled[0] = enabled;
                out.command[0] = {.4, 0, 10, 1, 0};
                out.zero_request_sequence[0] = request;
                out.zero_position[0] = requested_position;
                if (resume)
                    out.zero_resume_sequence[0] = request;
                out.clear_fault_sequence = clear;
                out.command_published_ns = shm_utils::MonotonicNs();
                ++out.command_sequence;
                ++shm.motor_sequence;
            }
            for (;;) {
                can_frame frame{};
                const auto count = read(peer, &frame, sizeof(frame));
                if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    break;
                Require(count == sizeof(frame), "invalid CAN frame");
                openrobot::Payload bytes{};
                std::copy_n(frame.data, 8, bytes.begin());
                if (bytes[0] == 0xC4 || bytes[0] == 0xC5)
                    send(frame.can_id, openrobot::EncodeProfile(45, 33, 1.296, 9, 1));
                else if (bytes[0] == 0xC1)
                    send(frame.can_id, {0xC1, 1});
                else if (bytes[0] == 0xC2)
                    send(frame.can_id, {0xC2});
                else if (bytes[0] == 0xC3) {
                    Require(frame.can_id == 0x141, "C3 sent to the wrong motor");
                    c3 = bytes;
                    ++c3_count;
                    c3_time = shm_utils::MonotonicNs();
                    echo_time = 0;
                    Require(bytes == openrobot::EncodeZeroPosition(requested_position),
                        "C3 did not carry the configured absolute position");
                    motor_position = openrobot::DecodeZeroPosition(bytes);
                    if (mode == ReplyMode::WrongEcho) {
                        ++bytes[6];
                        send(frame.can_id, bytes);
                    }
                } else if (bytes[0] == 0xC0) {
                    const auto command = openrobot::DecodeMitCommand(bytes, 45, 1, 33);
                    if (c3_time && !resume) {
                        Require(echo_time && shm_utils::MonotonicNs() - echo_time >= 20000000,
                            "C0 sent before C3 echo/hold completed");
                        Require(command.kp == 0 && command.kd == 0,
                            "old coordinate command sent before controller acknowledged C3");
                    } else if (command.kp > 0) {
                        ++active_commands;
                    }
                    if (mode != ReplyMode::NoFeedback || !c3_time)
                        send(frame.can_id, openrobot::EncodeFeedback({motor_position, 0, 0, 25}));
                }
            }
            if (c3_time && !echo_time && mode != ReplyMode::NoEcho &&
                mode != ReplyMode::WrongEcho && shm_utils::MonotonicNs() - c3_time >= 30000000) {
                send(0x141, c3);
                echo_time = shm_utils::MonotonicNs();
            }
            int status = 0;
            Require(waitpid(child, &status, WNOHANG) == 0, "HAL unexpectedly exited");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };
        const auto until = [&](auto predicate) {
            const auto deadline = shm_utils::MonotonicNs() + 2000000000ULL;
            while (!predicate()) {
                Require(shm_utils::MonotonicNs() < deadline, "C3 test timed out");
                pump();
            }
        };
        until([&] { return snapshot().profile_verified[0]; });
        enabled = true;
        until([&] { return active_commands > 2; });
        requested_position = -2.984513;
        request = 1;
        until([&] { return snapshot().zero_completed_sequence[0] == request; });
        auto state = snapshot();
        Require(state.coordinate_valid[0] && c3_count == 1, "C3 completion/one-shot failure");
        Require(std::abs(state.feedback[0].position - requested_position) < .0004,
            "fresh C3 feedback coordinate mismatch");
        Require(std::abs(state.zero_applied_position[0] - requested_position) < .0001,
            "C3 quantized coordinate mismatch");
        for (int i = 0; i < 35; ++i)
            pump();  // Keep sending stale-frame targets; HAL must reject all of them.
        resume = true;
        const auto before = active_commands;
        until([&] { return active_commands > before + 2; });
        for (auto failure : {ReplyMode::NoEcho, ReplyMode::WrongEcho, ReplyMode::NoFeedback}) {
            mode = ReplyMode::Normal;
            enabled = false;
            c3_time = echo_time = 0;
            ++clear;
            until([&] { return !snapshot().enabled[0] && snapshot().transport_fault == TransportFault::None; });
            enabled = true;
            until([&] { return snapshot().enabled[0]; });
            mode = failure;
            resume = false;
            ++request;
            const auto old_count = c3_count;
            until([&] { return snapshot().transport_fault != TransportFault::None; });
            state = snapshot();
            const auto expected = failure == ReplyMode::WrongEcho
                                      ? TransportFault::ZeroSetMismatch : TransportFault::ZeroSetTimeout;
            Require(state.transport_fault == expected && !state.coordinate_valid[0],
                "C3 failure did not invalidate calibration and latch the expected fault");
            Require(!state.enabled[0] && c3_count == old_count + 1,
                "C3 failure left output enabled or resent C3");
        }
        shm_utils::Stop(shm);
        kill(child, SIGTERM);
        waitpid(child, nullptr, 0);
        child = -1;
        close(peer);
        peer = -1;
        std::filesystem::remove_all(directory);
        std::cout << "PASS: signed C3, delayed echo, hold, fresh coordinates, stale-command gate, timeout and mismatch\n";
        return 0;
    } catch (const std::exception& error) {
        if (child > 0) {
            kill(child, SIGKILL);
            waitpid(child, nullptr, 0);
        }
        if (peer >= 0)
            close(peer);
        std::cerr << error.what() << " (diagnostics: " << directory << ")\n";
        return 1;
    }
}
