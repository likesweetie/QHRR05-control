#include "daemon/daemon.hpp"
#include "motor_can_protocol/openrobot_protocol.hpp"
#include "ipc/shm_utils.hpp"

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

        const auto timeout = [&](const char* key) {
            const double seconds = config[key].as<double>();
            if (!std::isfinite(seconds) || seconds <= 0 || seconds > 10)
                throw std::runtime_error(std::string(key) + " must be in (0, 10] seconds");
            return static_cast<std::uint64_t>(seconds * 1e9);
        };
        command_timeout_ns_ = timeout("command_timeout_sec");
        feedback_timeout_ns_ = timeout("feedback_timeout_sec");
        enable_timeout_ns_ = timeout("enable_timeout_sec");
        poll_released_ = config["poll_released"].as<bool>();
        state_.supported.fill(kCanSignals);

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
        motor_config_path_ = motor_path.lexically_normal().string();
        const auto motor = YAML::LoadFile(motor_config_path_);
        const auto mit = motor["mit"];
        velocity_max_ = mit["velocity_max_rad_s"].as<double>();
        torque_max_ = mit["torque_max_nm"].as<double>();
        kd_scale_ = mit["kd_scale"].as<unsigned>();
        configured_parameters_ = openrobot::EncodeProfile(velocity_max_, torque_max_,
            mit["kt_output_nm_a"].as<double>(), mit["gear_ratio"].as<double>(), kd_scale_);
        parameter_write_ = openrobot::EncodeProfileWrite(velocity_max_,
            torque_max_,
            mit["kt_output_nm_a"].as<double>(),
            mit["gear_ratio"].as<double>(),
            kd_scale_);
        expected_parameters_ = parameter_write_;
        expected_parameters_[0] = 0xC4;
        const unsigned input_kt = unsigned(parameter_write_[3]) | (unsigned(parameter_write_[4]) << 8);
        const unsigned ratio = unsigned(parameter_write_[5]) | (unsigned(parameter_write_[6]) << 8);
        gear_ratio_ = ratio * .01;
        const auto output_kt = static_cast<unsigned>(std::lround(input_kt * ratio / 100.0));
        expected_parameters_[3] = std::uint8_t(output_kt);
        expected_parameters_[4] = std::uint8_t(output_kt >> 8);
        std::cout << "[can_hal] startup profile from " << motor_config_path_
                  << ": V_MAX=" << velocity_max_ << " T_MAX=" << torque_max_
                  << " Kt_out requested=" << mit["kt_output_nm_a"].as<double>()
                  << " C5 Kt_input=" << input_kt * .001
                  << " C5 Kt_out=" << output_kt * .001
                  << " GR=" << ratio * .01 << " kd_scale=" << kd_scale_ << std::endl;
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

        for (int i = 0; i < 3; ++i)
            Send(i, {0xC2});

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
                        << interface_ << " | MIT C0 / RMD A1 | cycle " << snapshot.cycles
                        << '\n'
                        << "SID   profile mode       TX       RX   TX/s   RX/s   q[rad]  dq[rad/s]  Iq[A] temp invalid\n";

                    for (int i = 0; i < 3; ++i) {
                        text << std::hex << ids_[i] << std::dec << "   "
                             << (snapshot.profile_verified[i] ? "OK   " : "WAIT ") << "  "
                             << (snapshot.enabled[i]
                                        ? (snapshot.command[i].rmd_torque ? "RMD A1  " : "MIT C0  ")
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

        MotorData targets;
        try {
            while (!stopping) {
                auto& shm = *mapping_->get();
                {
                    shm_utils::TryLock lock(shm);
                    if (lock) {
                        if (shm.failed)
                            throw std::runtime_error("CAN SHM marked failed");

                        if (shm.stopping)
                            break;

                        targets = shm.motors[shm.motor_sequence % kShmBufferCount];
                    }
                }

                if (display_failed)
                    throw std::runtime_error("CAN statistics terminal output failed");

                Receive();
                const auto now = shm_utils::MonotonicNs();
                const bool requested = std::any_of(
                    targets.command_enabled.begin(), targets.command_enabled.end(), [](bool v) {
                        return v;
                    });
                if (targets.clear_fault_sequence != cleared_sequence_ && !requested) {
                    cleared_sequence_ = targets.clear_fault_sequence;
                    state_.transport_fault = TransportFault::None;
                    std::cout << "[can_hal] fault cleared; outputs remain released" << std::endl;
                }
                if (requested && (!targets.command_published_ns ||
                                     now - targets.command_published_ns > command_timeout_ns_))
                    Trip(TransportFault::CommandTimeout);
                for (int i = 0; i < 3; ++i) {
                    state_.valid[i] =
                        state_.received_ns[i] && now - state_.received_ns[i] <= feedback_timeout_ns_
                            ? kCanSignals
                            : 0;
                    if (state_.enable_requested[i] && state_.received_ns[i] < enable_since_[i] &&
                        now - enable_since_[i] > enable_timeout_ns_)
                        Trip(TransportFault::EnableTimeout);
                    if (zero_pending_[i] &&
                        now - zero_started_ns_[i] > std::min(feedback_timeout_ns_, enable_timeout_ns_))
                        Trip(TransportFault::ZeroSetTimeout);
                    if (!zero_pending_[i] && state_.enable_requested[i] && state_.received_ns[i] >= enable_since_[i] &&
                        now - state_.received_ns[i] > feedback_timeout_ns_)
                        Trip(TransportFault::FeedbackTimeout);
                }
                std::array<openrobot::Payload, 3> frames{};
                try {
                    for (int i = 0; i < 3; ++i) {
                        auto target = targets.command[i];
                        if (!std::isfinite(target.tau_ff))
                            throw std::runtime_error("non-finite MIT torque target");
                        if (target.rmd_torque) {
                            if (target.kp != 0 || target.kd != 0 || target.velocity != 0)
                                throw std::runtime_error("RMD torque command must have zero position/velocity gains");
                            if (state_.profile_verified[i])
                                frames[i] = openrobot::EncodeRmdTorque(target.tau_ff, motor_kt_output_[i],
                                    target.rmd_damping_constant);
                        } else {
                            target.tau_ff = std::clamp(target.tau_ff, -torque_max_, torque_max_);
                            frames[i] = openrobot::EncodeMitCommand(
                                target, velocity_max_, kd_scale_, torque_max_);
                        }
                    }
                } catch (const std::exception&) {
                    Trip(TransportFault::InvalidCommand);
                }
                for (int i = 0; i < 3; ++i) {
                    const bool command_enabled = targets.command_enabled[i] &&
                                                 state_.transport_fault == TransportFault::None;
                    if (!state_.profile_verified[i]) {
                        if (!startup_released_[i]) {
                            if (state_.cycles && state_.cycles % 100 == 0)
                                Send(i, {0xC2});
                        } else if (!profile_read_received_[i]) {
                            if (profile_read_started_ns_[i] &&
                                now - profile_read_started_ns_[i] > enable_timeout_ns_)
                                throw std::runtime_error("C4 startup read timeout for motor_id=" +
                                    std::to_string(ids_[i] - 0x140));
                            if (!profile_read_started_ns_[i] ||
                                (state_.cycles && state_.cycles % 100 == 0)) {
                                Send(i, {0xC4});
                                if (!profile_read_started_ns_[i])
                                    profile_read_started_ns_[i] = now;
                            }
                        } else if (!parameter_write_sent_[i]) {
                            Send(i, parameter_write_);
                            parameter_write_sent_[i] = true;
                            parameter_write_started_ns_[i] = now;
                            std::cout << "[can_hal] C5 sent once motor_id=" << (ids_[i] - 0x140)
                                      << "; waiting for C4 verification" << std::endl;
                        } else {
                            if (now - parameter_write_started_ns_[i] > enable_timeout_ns_)
                                throw std::runtime_error("C5/C4 verification timeout for motor_id=" +
                                    std::to_string(ids_[i] - 0x140));
                            if (state_.cycles && state_.cycles % 100 == 0)
                                Send(i, {0xC4});
                        }
                        continue;  // No C0/C1 until the written profile is verified.
                    }
                    if (state_.cycles && state_.cycles % 100 == 0) {
                        if (command_enabled && state_.enable_requested[i] && !state_.enabled[i])
                            Send(i, {0xC1});
                    }
                    if (command_enabled && !state_.enable_requested[i]) {
                        Send(i, {0xC1});
                        state_.enable_requested[i] = true;
                        enable_since_[i] = now;
                    } else if (!command_enabled && state_.enable_requested[i]) {
                        Send(i, {0xC2});
                        state_.enable_requested[i] = state_.enabled[i] = false;
                        rmd_mode_[i] = rmd_anchor_valid_[i] = rmd_anchor_pending_[i] = false;
                        last_mit_command_[i] = openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_);
                    }
                    if (command_enabled && !targets.command[i].rmd_torque && rmd_mode_[i]) {
                        // Explicitly re-enter MIT before any position command/C3.
                        Send(i, {0xC1});
                        rmd_mode_[i] = rmd_anchor_valid_[i] = rmd_anchor_pending_[i] = false;
                    }
                    if (command_enabled && state_.enabled[i] && targets.zero_request_sequence[i] &&
                        targets.zero_request_sequence[i] != zero_request_[i] &&
                        targets.zero_resume_sequence[i] != targets.zero_request_sequence[i]) {
                        if (zero_pending_[i]) {
                            Trip(TransportFault::InvalidCommand);
                            continue;
                        }
                        try {
                            zero_payload_[i] = openrobot::EncodeZeroPosition(targets.zero_position[i]);
                        } catch (const std::exception&) {
                            Trip(TransportFault::InvalidCommand);
                            continue;
                        }
                        zero_request_[i] = targets.zero_request_sequence[i];
                        zero_started_ns_[i] = now;
                        zero_ack_ns_[i] = 0;
                        zero_pending_[i] = true;
                        rmd_mode_[i] = rmd_anchor_valid_[i] = rmd_anchor_pending_[i] = false;
                        state_.coordinate_valid[i] = false;
                        Send(i, zero_payload_[i]);
                        std::cout << "[can_hal] C3 sent motor_id=" << (ids_[i] - 0x140)
                                  << " position=" << openrobot::DecodeZeroPosition(zero_payload_[i])
                                  << " rad sequence=" << zero_request_[i] << std::endl;
                    }
                    if (zero_pending_[i]) {
                        if (!command_enabled) {
                            zero_pending_[i] = false;
                        } else if (zero_ack_ns_[i] && now - zero_ack_ns_[i] >= kZeroHoldNs) {
                            // Request fresh feedback with zero gains, never an old-frame target.
                            Send(i, openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_));
                        }
                        continue;
                    }
                    if (state_.zero_completed_sequence[i] &&
                        targets.zero_resume_sequence[i] != state_.zero_completed_sequence[i]) {
                        if (command_enabled && state_.enabled[i])
                            Send(i, openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_));
                        continue;
                    }
                    if (command_enabled && state_.enabled[i]) {
                        if (targets.command[i].rmd_torque) {
                            if (!rmd_anchor_valid_[i]) {
                                if (!rmd_anchor_pending_[i]) {
                                    rmd_anchor_started_ns_[i] = now;
                                    rmd_anchor_sent_ns_[i] = 0;
                                    rmd_anchor_pending_[i] = true;
                                }
                                if (now - rmd_anchor_started_ns_[i] > feedback_timeout_ns_) {
                                    std::cerr << "[can_hal] RMD anchor timeout motor_id=" << (ids_[i] - 0x140) << '\n';
                                    Trip(TransportFault::FeedbackTimeout);
                                } else if (!rmd_anchor_sent_ns_[i] || now - rmd_anchor_sent_ns_[i] >= 10000000) {
                                    // Retry lost initial reads; never wait forever for one 9C.
                                    // Keep the last MIT effort until a fresh C0/9C pair establishes the C3 frame.
                                    rmd_anchor_sent_ns_[i] = shm_utils::MonotonicNs();
                                    Send(i, last_mit_command_[i][0] == 0xC0 ? last_mit_command_[i]
                                        : openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_));
                                    Send(i, {0x9C});
                                }
                                continue;
                            }
                            if (!rmd_mode_[i]) {
                                // Stop the MIT servo before selecting RMD current control.
                                // A1 itself starts current control; do not send another C1.
                                Send(i, {0xC2});
                                std::cout << "[can_hal] RMD torque motor_id=" << (ids_[i] - 0x140)
                                          << " Kt_out=" << motor_kt_output_[i]
                                          << " torque_range=+/-" << 33 * motor_kt_output_[i] << " Nm\n";
                                rmd_mode_[i] = true;
                            }
                        } else {
                            rmd_anchor_valid_[i] = rmd_anchor_pending_[i] = false;
                            last_mit_command_[i] = frames[i];
                        }
                        Send(i, frames[i]);
                    }
                    else if (poll_released_ && state_.profile_verified[i] &&
                             !state_.enable_requested[i])
                        Send(i, openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_));
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
    void Trip(TransportFault reason) {
        if (state_.transport_fault != TransportFault::None)
            return;
        state_.transport_fault = reason;
        std::cerr << "[can_hal] FAULT: " << TransportFaultName(reason)
                  << "; all outputs released, explicit clear required" << std::endl;
        for (int i = 0; i < 3; ++i) {
            if (state_.enable_requested[i] || state_.enabled[i])
                Send(i, {0xC2});
            state_.enable_requested[i] = state_.enabled[i] = false;
            rmd_mode_[i] = rmd_anchor_valid_[i] = rmd_anchor_pending_[i] = false;
            last_mit_command_[i] = openrobot::EncodeMitCommand({}, velocity_max_, kd_scale_, torque_max_);
        }
    }

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

            if (bytes[0] == 0xC2 && !parameter_write_sent_[i]) {
                startup_released_[i] = true;
            } else if (bytes[0] == 0xC4) {
                if (!profile_read_started_ns_[i])
                    continue;  // A stale read response cannot authorize startup.
                if (!state_.profile_verified[i] && !parameter_write_sent_[i]) {
                    profile_read_received_[i] = true;
                    // Accept only the exact configured C4 representation or its
                    // exact C5 round-trip representation. No numeric tolerance.
                    if (bytes != configured_parameters_ && bytes != expected_parameters_)
                        continue;  // The next cycle writes the requested profile once.
                }
                try {
                    if (state_.profile_verified[i])
                        openrobot::CheckProfile(bytes, verified_parameters_[i]);
                    else if (parameter_write_sent_[i])
                        openrobot::CheckProfile(bytes, expected_parameters_);
                } catch (const std::runtime_error& error) {
                    constexpr std::array<const char*, 3> axes{"HR", "HP", "KP"};
                    std::ostringstream message;
                    message << "interface=" << interface_ << " axis=" << axes[i]
                            << " motor_id=" << (ids_[i] - 0x140) << " SID=0x"
                            << std::hex << std::uppercase << ids_[i] << std::dec
                            << " motor_config=" << std::quoted(motor_config_path_)
                            << '\n' << error.what();
                    if (parameter_write_sent_[i] && !state_.profile_verified[i])
                        message << "\nC5 verification failed: firmware requires FOC OFF for ROM writes; "
                                   "a C2 acknowledgement alone does not confirm FOC OFF.";
                    throw std::runtime_error(message.str());
                }
                // Use the conversion read back from the motor, never a SHM default.
                motor_kt_output_[i] = (unsigned(bytes[3]) | (unsigned(bytes[4]) << 8)) * 0.001;
                if (!state_.profile_verified[i]) {
                    std::cout << "[can_hal] "
                              << (parameter_write_sent_[i] ? "C5 verified by C4" : "existing C4 matches motor YAML; C5 skipped")
                              << " motor_id=" << (ids_[i] - 0x140)
                              << " Kt_out=" << motor_kt_output_[i] << std::endl;
                    verified_parameters_[i] = bytes;
                }
                state_.profile_verified[i] = true;
            } else if (bytes[0] == 0xC1 && state_.enable_requested[i]) {
                if (bytes[1] != kd_scale_)
                    throw std::runtime_error("MIT C1 kd_scale mismatch");

                state_.enabled[i] = true;
            } else if (bytes[0] == 0xC3) {
                if (!zero_pending_[i] || bytes[6] != zero_payload_[i][6] ||
                    bytes[7] != zero_payload_[i][7]) {
                    state_.coordinate_valid[i] = false;
                    std::cerr << "[can_hal] C3 echo mismatch motor_id=" << (ids_[i] - 0x140)
                              << " actual=" << openrobot::DecodeZeroPosition(bytes) << " rad\n";
                    Trip(TransportFault::ZeroSetMismatch);
                } else if (!zero_ack_ns_[i]) {
                    zero_ack_ns_[i] = shm_utils::MonotonicNs();
                }
            } else if (bytes[0] == 0x9C && rmd_anchor_pending_[i]) {
                const auto received = shm_utils::MonotonicNs();
                if (state_.received_ns[i] < rmd_anchor_sent_ns_[i]) {
                    continue;  // Need the fresh C0 sample sent before this read request.
                }
                const auto feedback = openrobot::DecodeRmdFeedback(bytes, gear_ratio_);
                const double dt = (received - state_.received_ns[i]) / 1e9;
                rmd_position_[i] = state_.feedback[i].position +
                    .5 * (state_.feedback[i].velocity + feedback.velocity) * dt;
                rmd_last_angle_[i] = feedback.position;
                rmd_anchor_valid_[i] = true;
                rmd_anchor_pending_[i] = false;
            } else if (bytes[0] == 0xA1 && rmd_mode_[i] && rmd_anchor_valid_[i]) {
                const auto received = shm_utils::MonotonicNs();
                auto feedback = openrobot::DecodeRmdFeedback(bytes, gear_ratio_);
                constexpr double turn = 6.2831853071795864769;
                rmd_position_[i] += std::remainder(
                    feedback.position - rmd_last_angle_[i], turn / gear_ratio_);
                rmd_last_angle_[i] = feedback.position;
                feedback.position = rmd_position_[i];
                state_.feedback[i] = feedback;
                state_.motor_torque[i] = feedback.current * motor_kt_output_[i];
                state_.received_ns[i] = state_.measurement_ns[i] = received;
                ++state_.received_sequence[i];
            } else if (bytes[0] == 0xC0 && !rmd_mode_[i]) {
                if (!state_.profile_verified[i])
                    throw std::runtime_error("MIT feedback received before C4 verification");

                const auto received = shm_utils::MonotonicNs();
                if (zero_pending_[i] && (!zero_ack_ns_[i] || received - zero_ack_ns_[i] < kZeroHoldNs))
                    continue;
                state_.feedback[i] = openrobot::DecodeFeedback(bytes);
                state_.motor_torque[i] = state_.feedback[i].current * motor_kt_output_[i];
                state_.received_ns[i] = shm_utils::MonotonicNs();
                state_.measurement_ns[i] = state_.received_ns[i];
                ++state_.received_sequence[i];
                if (zero_pending_[i]) {
                    zero_pending_[i] = false;
                    state_.zero_completed_sequence[i] = zero_request_[i];
                    state_.zero_applied_position[i] = openrobot::DecodeZeroPosition(zero_payload_[i]);
                    state_.coordinate_valid[i] = true;
                    std::cout << "[can_hal] C3 confirmed motor_id=" << (ids_[i] - 0x140)
                              << " position=" << state_.zero_applied_position[i]
                              << " feedback=" << state_.feedback[i].position << " rad\n";
                }
            } else if (bytes[0] != 0xC2) {
                ++state_.invalid_frames[i];
                std::cerr << "[can_hal] unexpected MIT response opcode=" << int(bytes[0]) << '\n';
            }
        }
    }

    void Publish() {
        auto& shm = *mapping_->get();
        shm_utils::TryLock lock(shm);
        if (!lock)
            return;
        auto& next = shm_utils::Next(shm.motors, shm.motor_sequence);
        next.sample_dt = state_.sample_dt;
        next.running = state_.running;
        next.feedback_sequence = ++state_.feedback_sequence;
        next.feedback_published_ns = shm_utils::MonotonicNs();
        next.consumed_command_sequence = state_.consumed_command_sequence;
        next.last_tx_ns = state_.last_tx_ns;
        next.cycles = state_.cycles;
        next.socket_errors = state_.socket_errors;

        next.supported = state_.supported;
        next.valid = state_.valid;
        next.measurement_ns = state_.measurement_ns;
        next.transport_fault = state_.transport_fault;
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
        next.zero_completed_sequence = state_.zero_completed_sequence;
        next.zero_applied_position = state_.zero_applied_position;
        for (int i = 0; i < 3; ++i)
            if (zero_request_[i] || state_.transport_fault == TransportFault::ZeroSetMismatch)
                next.coordinate_valid[i] = state_.coordinate_valid[i];

        ++shm.motor_sequence;
        shm_utils::Notify(shm);
    }

    std::uint64_t command_timeout_ns_ = 0, feedback_timeout_ns_ = 0, enable_timeout_ns_ = 0;
    std::uint64_t cleared_sequence_ = 0;
    std::array<std::uint64_t, 3> enable_since_{};
    std::array<bool, 3> startup_released_{}, profile_read_received_{}, parameter_write_sent_{};
    std::array<std::uint64_t, 3> profile_read_started_ns_{}, parameter_write_started_ns_{};
    static constexpr std::uint64_t kZeroHoldNs = 20000000;
    std::array<std::uint64_t, 3> zero_request_{}, zero_started_ns_{}, zero_ack_ns_{};
    std::array<bool, 3> zero_pending_{};
    std::array<openrobot::Payload, 3> zero_payload_{};
    bool poll_released_ = false;
    int socket_ = -1, monitor_socket_ = -1;
    unsigned bus_load_bitrate_ = 0;
    std::string interface_;
    std::string motor_config_path_;
    std::array<canid_t, 3> ids_{};
    double velocity_max_ = 0, torque_max_ = 0;
    unsigned kd_scale_ = 0;
    openrobot::Payload configured_parameters_{}, parameter_write_{}, expected_parameters_{};
    std::array<openrobot::Payload, 3> verified_parameters_{};
    std::array<double, 3> motor_kt_output_{};  // Read back once via verified C4.
    double gear_ratio_ = 0;
    std::array<bool, 3> rmd_mode_{}, rmd_anchor_valid_{}, rmd_anchor_pending_{};
    std::array<double, 3> rmd_position_{}, rmd_last_angle_{};
    std::array<std::uint64_t, 3> rmd_anchor_sent_ns_{};
    std::array<std::uint64_t, 3> rmd_anchor_started_ns_{};
    std::array<openrobot::Payload, 3> last_mit_command_{};
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
            (std::string(argv[1]) == "--target" || std::string(argv[1]) == "--release"))
            throw std::runtime_error(
                "direct SHM motor commands were removed; use the Input interface");

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
