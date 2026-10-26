#include "daemon/daemon.hpp"
#include "motor_can/openrobot_protocol.hpp"
#include "shm_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <memory>
#include <net/if.h>
#include <net/if_arp.h>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace {
volatile std::sig_atomic_t stopping = 0;

void OnSignal(int) {
    stopping = 1;
}

std::uint64_t Period(double frequency, const char* name) {
    if (!std::isfinite(frequency) || frequency <= 0 || frequency > 1e6)
        throw std::runtime_error(std::string(name) + " must be finite and in (0, 1e6]");

    return static_cast<std::uint64_t>(1e9 / frequency);
}

class CanHal {
public:
    CanHal(const std::string& hal_config_path, const std::string& shm_name) {
        const auto config = YAML::LoadFile(hal_config_path);

        if (config["shm_name"])
            throw std::runtime_error(
                "HAL shm_name moved to daemon config; pass SHM_NAME as an argument");

        interface_ = config["interface"].as<std::string>();
        period_ns_ = Period(config["frequency_hz"].as<double>(), "frequency_hz");
        display_period_ns_ = Period(config["statistics_hz"].as<double>(), "statistics_hz");
        bus_load_bitrate_ = config["bus_load_bitrate"].as<unsigned>();

        if (!bus_load_bitrate_ || bus_load_bitrate_ > 1000000)
            throw std::runtime_error("bus_load_bitrate must be in [1, 1000000] bit/s");

        const auto ids = config["motor_ids"].as<std::vector<unsigned>>();

        if (ids.size() != 3 || interface_.empty() || interface_.size() >= IFNAMSIZ)
            throw std::runtime_error("CAN HAL requires an interface and exactly three motor IDs");

        for (int i = 0; i < 3; ++i) {
            if (!ids[i] || ids[i] > 254 ||
                std::find(ids.begin(), ids.begin() + i, ids[i]) != ids.begin() + i)
                throw std::runtime_error("motor IDs must be distinct and in [1, 254]");

            ids_[i] = 0x140 + ids[i];
        }

        const auto motor_path = std::filesystem::path(hal_config_path).parent_path() /
                                config["motor_config"].as<std::string>();
        const auto motor = YAML::LoadFile(motor_path.string());
        const auto mit = motor["mit"];
        velocity_max_ = mit["velocity_max_rad_s"].as<double>();
        kd_scale_ = mit["kd_scale"].as<unsigned>();
        expected_parameters_ = openrobot::EncodeProfile(velocity_max_,
            mit["torque_max_nm"].as<double>(),
            mit["kt_output_nm_a"].as<double>(),
            mit["gear_ratio"].as<double>(),
            kd_scale_);
        state_.sample_dt = period_ns_ / 1e9;

        const unsigned index = ::if_nametoindex(interface_.c_str());

        if (!index)
            throw std::runtime_error("CAN interface not found: " + interface_);

        socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

        if (socket_ < 0)
            shm_utils::Check(errno, "SocketCAN socket");

        try {
            ifreq request{};
            std::copy(interface_.begin(), interface_.end(), request.ifr_name);

            if (::ioctl(socket_, SIOCGIFHWADDR, &request) < 0)
                shm_utils::Check(errno, "CAN interface type");

            if (request.ifr_hwaddr.sa_family != ARPHRD_CAN)
                throw std::runtime_error("interface is not CAN: " + interface_);

            if (::ioctl(socket_, SIOCGIFFLAGS, &request) < 0)
                shm_utils::Check(errno, "CAN interface flags");

            if (!(request.ifr_flags & IFF_UP))
                throw std::runtime_error("CAN interface is down: " + interface_);

            std::array<can_filter, 3> filters{};

            for (int i = 0; i < 3; ++i)
                filters[i] = {ids_[i], CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};

            SetOption(SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters));
            const int receive_own = 0, loopback = 1;
            SetOption(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own, sizeof(receive_own));
            SetOption(SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
            const can_err_mask_t errors = CAN_ERR_MASK;
            SetOption(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors));
            sockaddr_can address{};
            address.can_family = AF_CAN;
            address.can_ifindex = int(index);

            if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
                shm_utils::Check(errno, "SocketCAN bind");

            // A receive-only socket observes every Classic CAN ID, including our
            // own traffic via local loopback exactly once, outside the control loop.
            monitor_socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

            if (monitor_socket_ < 0)
                shm_utils::Check(errno, "CAN bus monitor socket");

            const int overflow_reports = 1;

            if (::setsockopt(monitor_socket_,
                    SOL_SOCKET,
                    SO_RXQ_OVFL,
                    &overflow_reports,
                    sizeof(overflow_reports)) < 0 ||
                ::setsockopt(
                    monitor_socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
                shm_utils::Check(errno, "CAN bus monitor options");

            if (::bind(monitor_socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
                shm_utils::Check(errno, "CAN bus monitor bind");

            mapping_ = std::make_unique<shm_utils::Mapping>(shm_name);
            shm_utils::Lock lock(*mapping_->get());

            if (!mapping_->get()->can_enabled || mapping_->get()->stopping ||
                mapping_->get()->failed)
                throw std::runtime_error("CAN HAL requires an active CAN shared-memory session");
        } catch (...) {
            if (monitor_socket_ >= 0)
                ::close(monitor_socket_);

            monitor_socket_ = -1;
            ::close(socket_);
            socket_ = -1;
            throw;
        }
    }

    ~CanHal() {
        if (monitor_socket_ >= 0)
            ::close(monitor_socket_);

        if (socket_ >= 0)
            ::close(socket_);
    }

    void Run() {
        state_.running = true;
        const auto statistics_started_ns = shm_utils::MonotonicNs();

        for (int i = 0; i < 3; ++i) {
            Send(i, {0xC2});
            Send(i, {0xC4});
        }

        Publish();
        Daemon::NotifyReady();
        std::atomic<bool> display_done{false}, display_failed{false};
        std::atomic<std::uint64_t> display_drops{0};
        std::thread display([&] {
            const int output =
                ::open("/proc/self/fd/1", O_WRONLY | O_NONBLOCK | O_APPEND | O_CLOEXEC);

            if (output < 0) {
                display_failed = true;
                return;
            }

            std::array<std::uint64_t, 3> previous_tx{}, previous_rx{};
            auto previous_time = statistics_started_ns;
            auto next_display = previous_time + display_period_ns_;
            std::uint64_t bus_frames = 0, bus_bits = 0, error_reports = 0;
            std::uint32_t monitor_drops = 0;
            const bool managed_terminal = std::getenv("QHRR_TERMINAL_STATISTICS") != nullptr;
            const bool terminal = ::isatty(output);

            try {
                while (!display_done) {
                    const auto before_poll = shm_utils::MonotonicNs();
                    const auto remaining =
                        next_display > before_poll ? next_display - before_poll : 0;
                    const int timeout_ms =
                        int(std::min<std::uint64_t>(50, (remaining + 999999) / 1000000));
                    pollfd descriptor{monitor_socket_, POLLIN, 0};
                    const int ready = ::poll(&descriptor, 1, timeout_ms);

                    if (ready < 0 && errno == EINTR)
                        continue;

                    if (ready < 0 || descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                        throw std::runtime_error("CAN bus monitor poll failed");

                    for (unsigned batch = 0; batch < 4096; ++batch) {
                        can_frame frame{};
                        iovec payload{&frame, sizeof(frame)};
                        alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(std::uint32_t))]{};
                        msghdr message{};
                        message.msg_iov = &payload;
                        message.msg_iovlen = 1;
                        message.msg_control = ancillary;
                        message.msg_controllen = sizeof(ancillary);
                        const auto count = ::recvmsg(monitor_socket_, &message, MSG_DONTWAIT);

                        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                            break;

                        if (count < 0 && errno == EINTR)
                            continue;

                        if (count != sizeof(frame) || message.msg_flags & (MSG_TRUNC | MSG_CTRUNC))
                            throw std::runtime_error("CAN bus monitor received an invalid record");

                        for (auto* info = CMSG_FIRSTHDR(&message); info;
                             info = CMSG_NXTHDR(&message, info)) {
                            if (info->cmsg_level == SOL_SOCKET && info->cmsg_type == SO_RXQ_OVFL &&
                                info->cmsg_len >= CMSG_LEN(sizeof(monitor_drops)))
                                std::memcpy(&monitor_drops, CMSG_DATA(info), sizeof(monitor_drops));
                        }

                        if (frame.can_id & CAN_ERR_FLAG) {
                            ++error_reports;
                            continue;
                        }

                        // Classic CAN worst-case stuffing, ACK/EOF/intermission
                        // included. RTR has no data field even when DLC is nonzero.
                        const auto data_bytes = frame.can_id & CAN_RTR_FLAG ? 0 : frame.can_dlc;
                        bus_bits += (frame.can_id & CAN_EFF_FLAG ? 80 : 55) + 10 * data_bytes;
                        ++bus_frames;
                    }

                    const auto now = shm_utils::MonotonicNs();

                    if (now < next_display)
                        continue;

                    MotorData snapshot;
                    {
                        auto& shm = *mapping_->get();
                        shm_utils::Lock lock(shm);
                        snapshot = shm.motors[shm.motor_sequence % kShmBufferCount];
                    }

                    const double seconds = std::max(1e-9, (now - previous_time) / 1e9);
                    std::ostringstream text;
                    text
                        << (managed_terminal ? Daemon::kStatisticsBegin : "") << "CAN HAL | "
                        << interface_ << " | MIT position / tau_ff=0 | cycle " << snapshot.cycles
                        << '\n'
                        << "SID   profile mode       TX       RX   TX/s   RX/s   q[rad]  dq[rad/s]  Iq[A] temp invalid\n";

                    for (int i = 0; i < 3; ++i) {
                        text << std::hex << ids_[i] << std::dec << "   "
                             << (snapshot.profile_verified[i] ? "OK   " : "WAIT ") << "  "
                             << (snapshot.enabled[i]
                                        ? "ENABLED "
                                        : (snapshot.enable_requested[i] ? "ENTERING" : "RELEASED"))
                             << std::setw(9) << snapshot.tx_frames[i] << std::setw(9)
                             << snapshot.rx_frames[i] << std::fixed << std::setprecision(0)
                             << std::setw(7) << (snapshot.tx_frames[i] - previous_tx[i]) / seconds
                             << std::setw(7) << (snapshot.rx_frames[i] - previous_rx[i]) / seconds
                             << std::setprecision(3) << std::setw(9)
                             << snapshot.feedback[i].position << std::setw(11)
                             << snapshot.feedback[i].velocity << std::setw(8)
                             << snapshot.feedback[i].current << std::setw(5)
                             << snapshot.feedback[i].temperature << std::setw(8)
                             << snapshot.invalid_frames[i] << '\n';
                        previous_tx[i] = snapshot.tx_frames[i];
                        previous_rx[i] = snapshot.rx_frames[i];
                    }

                    const double load = 100.0 * bus_bits / (bus_load_bitrate_ * seconds);
                    constexpr int bar_width = 30;
                    const int filled =
                        int(std::lround(std::clamp(load, 0.0, 100.0) * bar_width / 100));
                    const char* color =
                        load >= 80 ? "\033[31m" : (load >= 60 ? "\033[33m" : "\033[32m");
                    text << '\n'
                         << "BUS LOAD est. " << (terminal ? color : "") << '['
                         << std::string(filled, '#') << std::string(bar_width - filled, '.') << "] "
                         << std::fixed << std::setprecision(1) << load << '%'
                         << (terminal ? "\033[0m" : "") << " | reference " << bus_load_bitrate_
                         << " bit/s" << (load > 100 ? " | OVER CAPACITY" : "")
                         << (monitor_drops ? " | INCOMPLETE" : "") << '\n'
                         << "Classic CAN / max stuffing | window " << std::setprecision(2)
                         << seconds * 1000 << " ms | frames " << bus_frames << " | bits "
                         << bus_bits << " | monitor drops=" << monitor_drops
                         << " | error reports=" << error_reports
                         << " (error/retry wire time excluded)\n";

                    text << "socket errors=" << snapshot.socket_errors
                         << " | display drops=" << display_drops
                         << " | delay evaluation: shm_logger only\n";

                    if (managed_terminal)
                        text << Daemon::kStatisticsEnd;
                    const auto bytes = text.str();
                    const auto count = ::write(output, bytes.data(), bytes.size());

                    if (count < 0 && errno != EAGAIN && errno != EINTR) {
                        display_failed = true;
                        break;
                    }

                    if (count > 0 && count != static_cast<ssize_t>(bytes.size()) &&
                        managed_terminal) {
                        display_failed = true;
                        break;
                    }

                    if (count != static_cast<ssize_t>(bytes.size()))
                        ++display_drops;

                    bus_frames = bus_bits = 0;
                    previous_time = now;
                    next_display = now + display_period_ns_;
                }

            } catch (const std::exception& error) {
                std::cerr << "[can_hal] statistics failed: " << error.what() << '\n';
                display_failed = true;
            }

            ::close(output);
        });

        std::exception_ptr failure;
        auto next_cycle = shm_utils::MonotonicNs();

        try {
            while (!stopping) {
                auto& shm = *mapping_->get();
                MotorData targets;
                {
                    shm_utils::Lock lock(shm);

                    if (shm.failed)
                        throw std::runtime_error("CAN SHM marked failed");

                    if (shm.stopping)
                        break;

                    targets = shm.motors[shm.motor_sequence % kShmBufferCount];
                }

                if (display_failed)
                    throw std::runtime_error("CAN statistics terminal output failed");

                Receive();
                std::array<openrobot::Payload, 3> frames{};

                // Validate the complete publication before transmitting any axis.
                for (int i = 0; i < 3; ++i)
                    frames[i] =
                        openrobot::EncodeMitCommand(targets.command[i], velocity_max_, kd_scale_);

                for (int i = 0; i < 3; ++i) {
                    const bool command_enabled = targets.command_enabled[i];

                    // Periodic discovery/enable requests, not a response-age check.
                    if (state_.cycles && state_.cycles % 100 == 0) {
                        if (!state_.profile_verified[i])
                            Send(i, {0xC4});
                        else if (command_enabled && state_.enable_requested[i] &&
                                 !state_.enabled[i])
                            Send(i, {0xC1});
                    }

                    if (command_enabled && !state_.enable_requested[i]) {
                        if (!state_.profile_verified[i])
                            throw std::runtime_error(
                                "enable requested before MIT C4 profile verification");

                        Send(i, {0xC1});
                        state_.enable_requested[i] = true;
                    } else if (!command_enabled && state_.enable_requested[i]) {
                        Send(i, {0xC2});
                        state_.enable_requested[i] = state_.enabled[i] = false;
                    }

                    if (command_enabled && state_.enabled[i])
                        Send(i, frames[i]);
                }

                state_.consumed_command_sequence = targets.command_sequence;
                ++state_.cycles;
                Publish();
                next_cycle += period_ns_;
                std::this_thread::sleep_until(
                    std::chrono::steady_clock::time_point(std::chrono::nanoseconds(next_cycle)));
            }
        } catch (...) {
            failure = std::current_exception();
        }

        for (int i = 0; i < 3; ++i) {
            try {
                Send(i, {0xC2});
                state_.enabled[i] = state_.enable_requested[i] = false;
            } catch (const std::exception& error) {
                std::cerr << "[can_hal] release failed: " << error.what() << '\n';
                failure = std::current_exception();
            }
        }

        state_.running = false;

        try {
            Publish();
        } catch (...) {
            failure = std::current_exception();
        }
        display_done = true;
        display.join();
        shm_utils::Stop(*mapping_->get(), bool(failure));

        if (failure)
            std::rethrow_exception(failure);
    }

private:
    void SetOption(int level, int option, const void* value, socklen_t size) {
        if (::setsockopt(socket_, level, option, value, size) < 0)
            shm_utils::Check(errno, "SocketCAN option");
    }

    void Send(int motor, const openrobot::Payload& bytes) {
        can_frame frame{};
        frame.can_id = ids_[motor];
        frame.can_dlc = 8;
        std::copy(bytes.begin(), bytes.end(), frame.data);
        ssize_t count;

        do {
            count = ::write(socket_, &frame, sizeof(frame));
        } while (count < 0 && errno == EINTR);

        if (count != sizeof(frame)) {
            ++state_.socket_errors;
            throw std::runtime_error("CAN write failed: " + std::string(std::strerror(errno)));
        }

        ++state_.tx_frames[motor];
        state_.last_tx_ns = shm_utils::MonotonicNs();
    }

    void Receive() {
        // A bounded drain keeps incoming traffic from monopolizing a control tick.
        for (unsigned batch = 0; batch < 256; ++batch) {
            can_frame frame{};
            const auto count = ::read(socket_, &frame, sizeof(frame));

            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;

            if (count < 0 && errno == EINTR)
                continue;

            if (count != sizeof(frame)) {
                ++state_.socket_errors;
                throw std::runtime_error("CAN read failed");
            }

            if (frame.can_id & CAN_ERR_FLAG) {
                ++state_.socket_errors;
                std::cerr << "[can_hal] CAN error frame: " << std::hex << frame.can_id << std::dec
                          << '\n';
                continue;
            }

            const auto found = std::find(ids_.begin(), ids_.end(), frame.can_id);

            if (found == ids_.end())
                continue;

            const int i = int(found - ids_.begin());
            ++state_.rx_frames[i];

            if (frame.can_dlc != 8) {
                ++state_.invalid_frames[i];
                std::cerr << "[can_hal] invalid DLC for motor " << i << '\n';
                continue;
            }

            openrobot::Payload bytes{};
            std::copy_n(frame.data, 8, bytes.begin());

            if (bytes[0] == 0xC4) {
                openrobot::CheckProfile(bytes, expected_parameters_);
                // Use the conversion read back from the motor, never a SHM default.
                motor_kt_output_[i] = (unsigned(bytes[3]) | (unsigned(bytes[4]) << 8)) * 0.001;
                state_.profile_verified[i] = true;
            } else if (bytes[0] == 0xC1 && state_.enable_requested[i]) {
                if (bytes[1] != kd_scale_)
                    throw std::runtime_error("MIT C1 kd_scale mismatch");

                state_.enabled[i] = true;
            } else if (bytes[0] == 0xC0) {
                if (!state_.profile_verified[i])
                    throw std::runtime_error("MIT feedback received before C4 verification");

                state_.feedback[i] = openrobot::DecodeFeedback(bytes);
                state_.motor_torque[i] = state_.feedback[i].current * motor_kt_output_[i];
                state_.received_ns[i] = shm_utils::MonotonicNs();
                ++state_.received_sequence[i];
            } else if (bytes[0] != 0xC2) {
                ++state_.invalid_frames[i];
                std::cerr << "[can_hal] unexpected MIT response opcode=" << int(bytes[0]) << '\n';
            }
        }
    }

    void Publish() {
        auto& shm = *mapping_->get();
        shm_utils::Lock lock(shm);
        auto& next = shm_utils::Next(shm.motors, shm.motor_sequence);
        next.sample_dt = state_.sample_dt;
        next.running = state_.running;
        next.feedback_sequence = ++state_.feedback_sequence;
        next.feedback_published_ns = shm_utils::MonotonicNs();
        next.consumed_command_sequence = state_.consumed_command_sequence;
        next.last_tx_ns = state_.last_tx_ns;
        next.cycles = state_.cycles;
        next.socket_errors = state_.socket_errors;

        next.feedback = state_.feedback;
        next.motor_torque = state_.motor_torque;
        next.received_ns = state_.received_ns;
        next.received_sequence = state_.received_sequence;
        next.tx_frames = state_.tx_frames;
        next.rx_frames = state_.rx_frames;
        next.invalid_frames = state_.invalid_frames;
        next.profile_verified = state_.profile_verified;
        next.enable_requested = state_.enable_requested;
        next.enabled = state_.enabled;

        ++shm.motor_sequence;
        shm_utils::Notify(shm);
    }

    int socket_ = -1, monitor_socket_ = -1;
    unsigned bus_load_bitrate_ = 0;
    std::string interface_;
    std::array<canid_t, 3> ids_{};
    double velocity_max_ = 0;
    unsigned kd_scale_ = 0;
    openrobot::Payload expected_parameters_{};  // Expected C4 response, local to HAL.
    std::array<double, 3> motor_kt_output_{};  // Read back once via verified C4.
    std::unique_ptr<shm_utils::Mapping> mapping_;
    MotorData state_{};
    std::uint64_t period_ns_ = 0, display_period_ns_ = 0;
};
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    std::signal(SIGHUP, OnSignal);
    std::signal(SIGPIPE, SIG_IGN);

    try {
        if (argc >= 2 &&
            (std::string(argv[1]) == "--target" || std::string(argv[1]) == "--release")) {
            const bool release = std::string(argv[1]) == "--release";

            if ((release && argc != 3) || (!release && argc != 8))
                throw std::runtime_error(
                    "usage: can_hal --target SHM AXIS Q DQ KP KD | --release SHM");

            shm_utils::Mapping mapping(argv[2]);
            auto& shm = *mapping.get();
            shm_utils::Lock lock(shm);

            if (shm.stopping || !shm.motors[shm.motor_sequence % kShmBufferCount].running)
                throw std::runtime_error("CAN HAL is not running");

            auto targets = shm.motors[shm.motor_sequence % kShmBufferCount];

            if (release) {
                targets.command_enabled.fill(false);
            } else {
                const auto number = [](const char* value) {
                    std::size_t length = 0;
                    const double parsed = std::stod(value, &length);

                    if (length != std::strlen(value) || !std::isfinite(parsed))
                        throw std::runtime_error("invalid numeric position target");

                    return parsed;
                };
                const double index = number(argv[3]);

                if (index < 0 || index > 2 || index != std::floor(index))
                    throw std::runtime_error("axis must be 0, 1 or 2");

                const auto& state = shm.motors[shm.motor_sequence % kShmBufferCount];

                if (!state.profile_verified[int(index)])
                    throw std::runtime_error("motor MIT profile has not been verified");

                MitCommand target{
                    number(argv[4]), number(argv[5]), number(argv[6]), number(argv[7]), 0};
                targets.command[int(index)] = target;
                targets.command_enabled[int(index)] = true;
            }

            ++targets.command_sequence;
            targets.command_published_ns = shm_utils::MonotonicNs();
            shm.motors[(shm.motor_sequence + 1) % kShmBufferCount] = targets;
            ++shm.motor_sequence;
            shm_utils::Notify(shm);

            return 0;
        }

        if (argc != 3)
            throw std::runtime_error("usage: can_hal HAL_CONFIG.yaml SHM_NAME");

        CanHal hal(argv[1], argv[2]);
        hal.Run();

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[can_hal] ERROR: " << error.what() << '\n';

        return 1;
    }
}
