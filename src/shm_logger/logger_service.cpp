#include "shm_logger/logger_service.hpp"
#include "ipc/shm_utils.hpp"
#include "daemon/daemon.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

namespace qhrr05::logging {
namespace {
double Positive(const YAML::Node& n, const char* key, double low, double high) {
    const double value = n[key].as<double>();
    if (!std::isfinite(value) || value < low || value > high)
        throw std::runtime_error(std::string("logger config out of range: ") + key);
    return value;
}

std::chrono::nanoseconds Nanos(double seconds) {
    return std::chrono::nanoseconds(static_cast<std::int64_t>(seconds * 1e9));
}
}

std::string ControlSocketPath(const std::string& shm_name) {
    if (shm_name.size() < 2 || shm_name[0] != '/' || shm_name.find('/', 1) != std::string::npos)
        throw std::runtime_error("invalid logger SHM name");
    return "/tmp/qhrr05-log-" + std::to_string(::getuid()) + "-" + shm_name.substr(1) + ".sock";
}

LoggerService::LoggerService(
    const std::string& config_path, const std::string& shm_name, const std::string& daemon_config)
    : config_path_(fs::canonical(config_path)),
      daemon_config_(daemon_config.empty() ? fs::path{} : fs::canonical(daemon_config)),
      socket_path_(ControlSocketPath(shm_name)), shm_name_(shm_name) {
    config_ = YAML::LoadFile(config_path_.string())["logging"];
    if (!config_ || !config_.IsMap())
        throw std::runtime_error("logger config requires logging map");
    auto_start_ = config_["enabled"].as<bool>() && config_["auto_start"].as<bool>();
    runtime_mode_ = config_["runtime_mode"].as<std::string>();
    if (runtime_mode_ != "simulation" && runtime_mode_ != "hardware")
        throw std::runtime_error("logger runtime_mode must be simulation or hardware");
    label_ = config_["session_label"].as<std::string>();
    if (label_.empty() || label_.size() > 64 ||
        label_.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
            std::string::npos)
        throw std::runtime_error("logger session_label must be 1..64 filename characters");

    rate_hz_ = Positive(config_, "sampling_rate_hz", 1, 10000);
    health_hz_ = Positive(config_, "health_rate_hz", 1, 100);
    display_sec_ = Positive(config_, "display_interval_sec", 1.01, 60);
    attach_timeout_sec_ = Positive(config_, "attach_timeout_sec", .01, 600);
    pool_slots_ = static_cast<std::size_t>(Positive(config_, "pool_slots", 1, 65536));
    continuous_enabled_ = config_["continuous"]["enabled"].as<bool>();
    flight_enabled_ = config_["flight_recorder"]["enabled"].as<bool>();
    if (!continuous_enabled_ && !flight_enabled_)
        throw std::runtime_error("logger needs at least one enabled recording sink");
    rotation_sec_ = Positive(config_["continuous"], "rotation_seconds", .01, 86400);
    segment_bytes_ = static_cast<std::uint64_t>(
        Positive(config_["continuous"], "segment_mib", .001, 65536) * (1ULL << 20));
    pre_sec_ = Positive(config_["flight_recorder"], "pre_trigger_seconds", 0, 120);
    post_sec_ = Positive(config_["flight_recorder"], "post_trigger_seconds", 0, 120);
    ring_bytes_ = static_cast<std::uint64_t>(
        Positive(config_["flight_recorder"], "capacity_mib", .001, 1024) * (1ULL << 20));
    stale_sec_ = Positive(config_["health"], "stale_seconds", .001, 60);
    can_timeout_sec_ = Positive(config_["health"], "can_timeout_seconds", .001, 60);
    const auto storage = config_["storage"];
    const auto maximum =
        static_cast<std::uint64_t>(Positive(storage, "maximum_gib", .001, 1048576) * (1ULL << 30));
    const auto minimum = static_cast<std::uint64_t>(
        Positive(storage, "minimum_free_gib", 0, 1048576) * (1ULL << 30));
    storage_ = std::make_unique<SessionStorage>(
        config_path_.parent_path() / config_["output_dir"].as<std::string>(),
        maximum,
        minimum,
        storage["protect_fault_records"].as<bool>());
    event_code_.reserve(64);
    event_detail_.reserve(2048);
}

LoggerService::~LoggerService() {
    try {
        Stop("interrupted");
    } catch (const std::exception& e) {
        std::cerr << "[shm_logger] finalization ERROR: " << e.what() << '\n';
    }
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);

        struct stat info {};

        if (::lstat(socket_path_.c_str(), &info) == 0 && info.st_dev == socket_device_ &&
            info.st_ino == socket_inode_)
            ::unlink(socket_path_.c_str());
    }
}

void LoggerService::Attach() {
    reader_ = std::make_unique<SnapshotReader>(shm_name_);
    health_.state = "READY";
    std::cout << "[shm_logger] attached " << shm_name_ << " ABI=" << kShmAbiVersion << '\n';
}

void LoggerService::OpenSocket() {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path_.string().size() >= sizeof(address.sun_path))
        throw std::runtime_error("logger control socket path is too long");
    std::strcpy(address.sun_path, socket_path_.c_str());

    struct stat previous {};

    if (::lstat(socket_path_.c_str(), &previous) == 0) {
        if (!S_ISSOCK(previous.st_mode) || previous.st_uid != ::getuid())
            throw std::runtime_error("refusing to replace unowned/non-socket logger path");
        const int probe = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (probe < 0)
            shm_utils::Check(errno, "logger control probe");
        const int connected =
            ::connect(probe, reinterpret_cast<sockaddr*>(&address), sizeof(address));
        const int error = errno;
        ::close(probe);
        if (connected == 0 || error != ECONNREFUSED)
            throw std::runtime_error("logger control socket is already active or inaccessible");
        std::cerr << "[shm_logger] removing stale owned control socket: " << socket_path_ << '\n';
        if (::unlink(socket_path_.c_str()) != 0)
            shm_utils::Check(errno, "logger stale socket unlink");
    } else if (errno != ENOENT)
        shm_utils::Check(errno, "logger socket lstat");

    socket_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0)
        shm_utils::Check(errno, "logger control socket");
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        shm_utils::Check(errno, "logger control bind");
    if (::chmod(socket_path_.c_str(), 0600) != 0 || ::listen(socket_fd_, 8) != 0)
        shm_utils::Check(errno, "logger socket permissions/listen");

    struct stat info {};

    if (::lstat(socket_path_.c_str(), &info) != 0)
        shm_utils::Check(errno, "logger socket identity");
    socket_device_ = info.st_dev;
    socket_inode_ = info.st_ino;
    std::cout << "[shm_logger] control socket: " << socket_path_ << '\n';
}

void LoggerService::Start() {
    if (!config_["enabled"].as<bool>())
        throw std::runtime_error("logging.enabled is false");
    if (collecting_ && !sampler_failed_ && health_.state != "FAULT")
        throw std::runtime_error("recording is already active");
    if (collecting_ || state_channel_ || continuous_ || flight_ || sampler_.joinable())
        Stop("recovery");
    if (!reader_)
        Attach();
    if (!reader_->SameObject())
        throw std::runtime_error("SHM was replaced; restart logger to attach explicitly");

    health_.state = "RECOVERING";
    storage_->Enforce({});
    storage_writable_ = true;
    YAML::Node sources =
        config_["sources"] ? YAML::Clone(config_["sources"]) : YAML::Node(YAML::NodeType::Map);
    LogFrame initial;
    if (!reader_->Read(initial))
        throw std::runtime_error("SHM busy during session initialization; retry start");
    reader_->ResetCounters();
    auxiliary_failures_ = 0;
    sources["without_can"] = !initial.can_enabled;
    storage_->Begin(
        label_, config_path_, daemon_config_, runtime_mode_, shm_name_, rate_hz_, sources);
    active_file_ = storage_->Directory() / "recording.mcap";
    segment_number_ = 0;
    ++session_number_;
    start_ns_ = segment_start_ns_ = shm_utils::MonotonicNs();
    health_previous_ns_ = start_ns_;
    health_previous_input_ = health_previous_output_ = 0;
    have_observed_ = false;
    source_fault_ = sampler_failed_ = false;
    fault_since_ns_ = 0;
    accepted_ = rejected_ = last_success_ = schedule_skips_ = blocked_ = 0;
    source_regressions_ = invalid_times_ = 0;
    for (auto& gap : source_gaps_)
        gap = 0;
    health_ = LoggerHealth{};
    health_.state = "RECORDING";
    active_triggers_.clear();
    dump_requests_.clear();
    last_recoveries_ = reader_->Counters().recoveries;

    if (continuous_enabled_) {
        if (fs::exists(active_file_))
            throw std::runtime_error("new session recording path already exists");
        continuous_ = DataTamer2::MCAPSink::create(
            active_file_.string(), config_["continuous"]["compression"].as<bool>());
        // Rotation is managed here for both byte and time limits, with protected active paths.
        continuous_->sink().setMaxTimeBeforeReset(std::chrono::seconds(0));
    }
    if (flight_enabled_) {
        DataTamer2::MCAPRingOptions options;
        options.filepath = (storage_->Directory() / "faults/fault.mcap").string();
        options.window = Nanos(pre_sec_);
        options.capacity_bytes = ring_bytes_;
        options.compression = config_["continuous"]["compression"].as<bool>();
        flight_ = DataTamer2::MCAPRingSink::create(options);
        flight_->sink().setDumpCallback([this](const DataTamer2::MCAPRingDump& dump) {
            std::lock_guard<std::mutex> lock(dump_mutex_);
            if (dumps_.size() < 8)
                dumps_.push_back(dump);
            else {
                ++callback_drops_;
                std::cerr << "[shm_logger] ERROR: flight dump report queue overflow; total="
                          << callback_drops_ << '\n';
            }
        });
    }

    state_channel_ = DataTamer2::LogChannel::create("qhrr/state");
    health_channel_ = DataTamer2::LogChannel::create("qhrr/health");
    event_channel_ = DataTamer2::LogChannel::create("qhrr/events");
    session_channel_ = DataTamer2::LogChannel::create("qhrr/session");
    simulation_id_ = RegisterState(*state_channel_, frame_, runtime_mode_ == "simulation");
    health_channel_->registerValue("logger/capture_ns", &health_.capture_ns);
    health_channel_->registerValue("logger/snapshot_attempts", &health_.snapshot_attempts);
    health_channel_->registerValue("logger/snapshot_successes", &health_.snapshot_successes);
    health_channel_->registerValue("logger/snapshot_failures", &health_.snapshot_failures);
    health_channel_->registerValue("logger/shm_lock_misses", &health_.shm_lock_misses);
    health_channel_->registerValue("logger/mutex_recoveries", &health_.mutex_recoveries);
    health_channel_->registerValue("logger/abi_failures", &health_.abi_failures);
    health_channel_->registerValue("logger/source_regressions", &health_.source_regressions);
    health_channel_->registerValue("logger/invalid_timestamps", &health_.invalid_timestamps);
    health_channel_->registerValue(
        "logger/last_successful_snapshot_ns", &health_.last_successful_snapshot_ns);
    health_channel_->registerValue("logger/schedule_skips", &health_.schedule_skips);
    health_channel_->registerValue("logger/mutex_wait_average_us", &health_.mutex_wait_average_us);
    health_channel_->registerValue("logger/mutex_wait_max_us", &health_.mutex_wait_max_us);
    health_channel_->registerValue("logger/mutex_hold_average_us", &health_.mutex_hold_average_us);
    health_channel_->registerValue("logger/mutex_hold_max_us", &health_.mutex_hold_max_us);
    health_channel_->registerValue("logger/input_rate_hz", &health_.input_rate_hz);
    health_channel_->registerValue("logger/output_rate_hz", &health_.output_rate_hz);
    health_channel_->registerValue("logger/drop_rate", &health_.drop_rate);
    health_channel_->registerValue("logger/datatamer_attempts", &health_.datatamer_attempts);
    health_channel_->registerValue("logger/datatamer_accepted", &health_.datatamer_accepted);
    health_channel_->registerValue("logger/pool_exhausted", &health_.pool_exhausted);
    health_channel_->registerValue("logger/oversize", &health_.oversize);
    health_channel_->registerValue(
        "logger/serialization_failures", &health_.serialization_failures);
    health_channel_->registerValue("logger/blocked_snapshots", &health_.blocked_snapshots);
    health_channel_->registerValue("logger/sink_delivered", &health_.sink_delivered);
    health_channel_->registerValue("logger/sink_errors", &health_.sink_errors);
    health_channel_->registerValue("logger/queue_high_water", &health_.queue_high_water);
    health_channel_->registerValue("logger/disk_available_bytes", &health_.disk_available_bytes);
    health_channel_->registerValue("logger/flight_dump_failures", &health_.flight_dump_failures);
    health_channel_->registerValue("logger/triggers_accepted", &health_.triggers_accepted);
    health_channel_->registerValue("logger/triggers_rejected", &health_.triggers_rejected);
    health_channel_->registerValue("logger/flight_evictions", &health_.flight_evictions);
    health_channel_->registerValue("logger/flight_report_drops", &health_.flight_report_drops);
    health_channel_->registerValue("logger/flight_stored_bytes", &health_.flight_stored_bytes);
    health_channel_->registerValue(
        "logger/flight_stored_snapshots", &health_.flight_stored_snapshots);
    health_channel_->registerValue("logger/state", &health_.state);
    health_channel_->registerValue("logger/source_sequence_gaps", &health_.source_sequence_gaps);
    health_channel_->registerValue("logger/source_age_ms", &health_.source_age_ms);
    health_channel_->registerValue("logger/source_valid", &health_.source_valid);
    health_channel_->registerValue("logger/last_error", &health_.last_error);
    event_channel_->registerValue("event/time_ns", &event_ns_);
    event_channel_->registerValue("event/id", &event_id_);
    event_channel_->registerValue("event/code", &event_code_);
    event_channel_->registerValue("event/detail", &event_detail_);
    event_channel_->registerValue("event/dump_accepted", &event_accepted_);
    session_text_ = Json(storage_->Metadata());
    session_channel_->registerValue("metadata/json", &session_text_);
    for (const auto& channel :
        {state_channel_, health_channel_, event_channel_, session_channel_}) {
        channel->setPoolCapacity(channel == state_channel_ ? pool_slots_ : 64);
        channel->setPayloadCapacity(channel == session_channel_ ? 65536 : 8192);
        if (continuous_)
            channel->addDataSink(continuous_);
        if (flight_)
            channel->addDataSink(flight_);
        channel->startLogging();
    }
    const auto time = std::chrono::nanoseconds(start_ns_);
    if (!DataTamer2::accepted(session_channel_->tryTakeSnapshot(time)))
        throw std::runtime_error("session metadata snapshot was rejected");
    // Schemas and field units are inspectable without decoding a single sample.
    std::ofstream schema_file(storage_->Directory() / "schemas.txt");
    for (const auto& channel : {state_channel_, health_channel_, event_channel_, session_channel_})
        schema_file << DataTamer2::ToStr(channel->getSchema()) << '\n';
    schema_file.close();
    if (!schema_file)
        throw std::runtime_error("cannot save channel schemas");
    YAML::Node units;
    units["motor/*/position_rad"] = "rad";
    units["motor/*/velocity_rad_s"] = "rad/s";
    units["motor/*/current_a"] = "A";
    units["motor/*/torque_nm"] = "Nm";
    units["*_ns"] = "CLOCK_MONOTONIC nanoseconds";
    units["model/mass"] = "kg m^2 (joint space)";
    units["model/jacobian"] = "m/rad; row-major XYZ by HR/HP/KP";
    units["simulation/ground_truth/contact_force"] = "N, world frame, force on foot";
    units["control/magnetic/estimated_force"] = "N, world frame, external force on foot";
    WriteJson(storage_->Directory() / "units.json", units);

    Event("RECORDING_STARTED", storage_->Directory().string());
    collecting_ = true;
    sampler_ = std::thread(&LoggerService::SampleLoop, this);
}

void LoggerService::SampleLoop() noexcept {
    try {
        const auto period = static_cast<std::uint64_t>(1e9 / rate_hz_);
        auto next = shm_utils::MonotonicNs();
        std::array<std::uint64_t, 6> previous{};
        std::array<std::uint64_t, 4> publication{};
        bool gt_enabled = false;
        while (collecting_) {
            if (!source_fault_ && reader_->Read(frame_)) {
                frame_.simulation_valid = runtime_mode_ == "simulation" &&
                                          frame_.simulation_sequence != 0 &&
                                          frame_.simulation.published_ns != 0;
                if (simulation_id_ && frame_.simulation_valid != gt_enabled) {
                    if (!state_channel_->trySetEnabled(*simulation_id_, frame_.simulation_valid))
                        throw std::runtime_error("simulation schema registration became invalid");
                    gt_enabled = frame_.simulation_valid;
                }
                const std::array<std::uint64_t, 6> sequences{frame_.motor_sequence,
                    frame_.model_sequence,
                    frame_.control.command_sequence,
                    frame_.simulation_sequence,
                    frame_.motor.command_sequence,
                    frame_.motor.feedback_sequence};
                for (int i = 0; i < 6; ++i) {
                    if (previous[i] && sequences[i] < previous[i])
                        ++source_regressions_;
                    if (previous[i] && sequences[i] > previous[i] + 1)
                        source_gaps_[i] += sequences[i] - previous[i] - 1;
                    previous[i] = sequences[i];
                }
                const std::array<std::uint64_t, 4> times{frame_.motor.feedback_published_ns,
                    frame_.model.published_ns,
                    frame_.control.published_ns,
                    frame_.simulation.published_ns};
                for (int i = 0; i < 4; ++i) {
                    if (times[i] > frame_.capture_ns ||
                        (publication[i] && times[i] < publication[i]))
                        ++invalid_times_;
                    publication[i] = times[i];
                }

                const auto result =
                    state_channel_->tryTakeSnapshot(std::chrono::nanoseconds(frame_.capture_ns));
                if (DataTamer2::accepted(result)) {
                    ++accepted_;
                    last_success_ = frame_.capture_ns;
                } else {
                    ++rejected_;
                    if (result == DataTamer2::SnapshotResult::blocked)
                        ++blocked_;
                }
                if (latest_mutex_.try_lock()) {
                    latest_ = frame_;
                    latest_mutex_.unlock();
                }
            }
            next += period;
            const auto now = shm_utils::MonotonicNs();
            if (next <= now) {
                const auto missed = (now - next) / period + 1;
                schedule_skips_ += missed;
                next += missed * period;
            }
            const timespec wake{
                static_cast<time_t>(next / 1000000000ULL), static_cast<long>(next % 1000000000ULL)};
            int result;
            do {
                result = ::clock_nanosleep(kShmClock, TIMER_ABSTIME, &wake, nullptr);
            } while (result == EINTR && collecting_);
            if (result && result != EINTR)
                shm_utils::Check(result, "logger scheduler");
        }
    } catch (const std::exception& error) {
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            sampler_error_ = error.what();
        }
        sampler_failed_ = true;
    }
}

void LoggerService::Event(const std::string& code, const std::string& detail) {
    event_ns_ = shm_utils::MonotonicNs();
    ++event_id_;
    event_code_ = code;
    event_detail_ = detail.substr(0, 2048);
    std::cout << "[shm_logger] " << code << ": " << event_detail_ << '\n';
    if (event_channel_ &&
        (collecting_ || code == "RECORDING_STARTED" || code == "RECORDING_STOPPING") &&
        !DataTamer2::accepted(event_channel_->tryTakeSnapshot(std::chrono::nanoseconds(event_ns_))))
        ++auxiliary_failures_;
}

bool LoggerService::Trigger(const std::string& code) {
    const bool accepted = collecting_ && storage_writable_ && flight_ &&
                          flight_->sink().requestDump(Nanos(post_sec_));
    if (accepted) {
        ++health_.triggers_accepted;
        if (dump_requests_.size() < 8)
            dump_requests_.emplace_back(shm_utils::MonotonicNs(), code);
        else
            ++auxiliary_failures_;
    } else
        ++health_.triggers_rejected;
    event_accepted_ = accepted;
    Event(code,
        accepted ? "flight dump accepted"
                 : "flight dump rejected (storage unavailable, disabled, or pending)");
    event_accepted_ = false;
    return accepted;
}

void LoggerService::Fault(const std::string& message) {
    if (health_.last_error == message)
        return;
    health_.last_error = message;
    if (!fault_since_ns_)
        fault_since_ns_ = shm_utils::MonotonicNs();
    health_.state = flight_ && collecting_ ? "DEGRADED" : "FAULT";
    std::cerr << "[shm_logger] ERROR: " << message << '\n';
    Event("LOGGER_ERROR", message);
    if (continuous_)
        continuous_->stop();
}

void LoggerService::CheckTriggers(const LogFrame& f) {
    std::set<std::string> present;
    const auto now = shm_utils::MonotonicNs();
    if (now - start_ns_ > 1000000000ULL) {
        for (int i = 0; i < 4; ++i) {
            if (i == 3 && runtime_mode_ == "hardware")
                continue;
            if (!health_.source_valid[i] || health_.source_age_ms[i] > stale_sec_ * 1000)
                present.insert("SHM_STALE");
        }
        if (f.can_enabled) {
            for (int i = 0; i < 3; ++i)
                if (!f.motor.received_ns[i] || f.motor.received_ns[i] > now ||
                    now - f.motor.received_ns[i] > std::uint64_t(can_timeout_sec_ * 1e9))
                    present.insert("CAN_TIMEOUT");
        }
        if (f.control.mode == 2 && !f.control.magnetic.observer_valid)
            present.insert("RFOB_INVALID");
    }
    if (f.failed)
        present.insert("SAFETY_STOP");
    if (f.model.published_ns) {
        for (int i = 0; i < 3; ++i) {
            const double q = f.motor.feedback[i].position + f.motor.position_offset[i];
            if (f.model.joint_max[i] > f.model.joint_min[i] &&
                (q < f.model.joint_min[i] || q > f.model.joint_max[i]))
                present.insert("JOINT_LIMIT");
        }
    }
    for (const auto& trigger : present)
        if (!active_triggers_.count(trigger))
            Trigger(trigger);
    active_triggers_ = std::move(present);
    if (f.control.mode == 3 && f.control.tracking_ik_failed &&
        (!have_observed_ || last_observed_.control.mode != 3 ||
            !last_observed_.control.tracking_ik_failed ||
            f.control.active_motion_sequence != last_observed_.control.active_motion_sequence))
        Event("TRACKING_IK_HOLD", "error_m=" + std::to_string(f.control.tracking_ik_error_m));

    if (have_observed_) {
        if (f.control.mode != last_observed_.control.mode)
            Event("MODE_CHANGED", std::to_string(f.control.mode));
        if (f.control.magnetic.event_sequence != last_observed_.control.magnetic.event_sequence)
            Event("MAGNETIC_EVENT", MagneticPhaseName(f.control.magnetic.phase));
        if (f.simulation_valid &&
            f.simulation.contact_event_sequence != last_observed_.simulation.contact_event_sequence)
            Event("CONTACT_EVENT", f.simulation.contact ? "contact" : "release");
    }
    last_observed_ = f;
    have_observed_ = true;
}

void LoggerService::UpdateHealth() {
    const auto now = shm_utils::MonotonicNs();
    health_.capture_ns = now;
    if (reader_) {
        const auto c = reader_->Counters();
        health_.snapshot_attempts = c.attempts;
        health_.snapshot_successes = accepted_;
        health_.snapshot_failures = rejected_.load() + auxiliary_failures_;
        health_.shm_lock_misses = c.lock_misses;
        health_.mutex_recoveries = c.recoveries;
        health_.mutex_wait_average_us = c.wait_average_us;
        health_.mutex_wait_max_us = c.wait_max_us;
        health_.mutex_hold_average_us = c.hold_average_us;
        health_.mutex_hold_max_us = c.hold_max_us;
        health_.source_regressions = source_regressions_;
        health_.invalid_timestamps = invalid_times_;
        health_.last_successful_snapshot_ns = last_success_;
        health_.schedule_skips = schedule_skips_;
        health_.blocked_snapshots = blocked_;
        for (int i = 0; i < 6; ++i)
            health_.source_sequence_gaps[i] = source_gaps_[i];
        if (state_channel_ && !source_fault_ && !reader_->SameObject()) {
            source_fault_ = true;
            Event("SHM_REPLACED", "source inode changed/unlinked; collection suspended");
            Fault("SHM replaced: restart logger before collecting the new object");
        }
        if (c.recoveries != last_recoveries_) {
            Trigger("MUTEX_RECOVERY");
            last_recoveries_ = c.recoveries;
        }
    }
    if (sampler_failed_) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        Fault("snapshot collector failed: " + sampler_error_);
    }
    LogFrame current;
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        current = latest_;
    }
    const std::array<std::uint64_t, 4> published{current.motor.feedback_published_ns,
        current.model.published_ns,
        current.control.published_ns,
        current.simulation.published_ns};
    for (int i = 0; i < 4; ++i) {
        health_.source_valid[i] =
            published[i] && published[i] <= now && !(i == 3 && runtime_mode_ == "hardware");
        health_.source_age_ms[i] = health_.source_valid[i]
                                       ? (now - published[i]) / 1e6
                                       : std::numeric_limits<double>::quiet_NaN();
    }
    const double seconds = (now - health_previous_ns_) / 1e9;
    if (seconds > 0 && health_previous_ns_) {
        const auto input = current.motor.command_sequence;
        health_.input_rate_hz =
            input >= health_previous_input_ ? (input - health_previous_input_) / seconds : 0;
        health_.output_rate_hz = (accepted_.load() - health_previous_output_) / seconds;
        health_previous_input_ = input;
        health_previous_output_ = accepted_;
    }
    health_previous_ns_ = now;
    health_.drop_rate =
        health_.snapshot_attempts
            ? double(health_.shm_lock_misses + rejected_.load()) / health_.snapshot_attempts
            : 0;
    health_.datatamer_attempts = health_.datatamer_accepted = health_.pool_exhausted =
        health_.oversize = health_.serialization_failures = 0;
    for (const auto& channel :
        {state_channel_, health_channel_, event_channel_, session_channel_}) {
        if (!channel)
            continue;
        health_.datatamer_attempts += channel->snapshotAttempts();
        health_.datatamer_accepted += channel->snapshotsAccepted();
        health_.pool_exhausted += channel->poolExhausted();
        health_.oversize += channel->droppedOversize();
        health_.serialization_failures += channel->serializeFailures();
    }
    health_.sink_delivered = health_.sink_errors = health_.queue_high_water = 0;
    for (const std::shared_ptr<DataTamer2::SinkWorker>& sink :
        {std::static_pointer_cast<DataTamer2::SinkWorker>(continuous_),
            std::static_pointer_cast<DataTamer2::SinkWorker>(flight_)}) {
        if (!sink)
            continue;
        health_.sink_delivered += sink->delivered();
        health_.sink_errors += sink->errors();
        health_.queue_high_water = std::max(health_.queue_high_water, sink->queueHighWater());
        if (sink->errors() && health_.last_error.empty())
            Fault("sink write failed: " + sink->lastError());
    }
    if (fault_since_ns_ && now - fault_since_ns_ > 5000000000ULL)
        health_.state = "FAULT";
    if (collecting_ && current.capture_ns)
        CheckTriggers(current);
    if (flight_) {
        const auto stats = flight_->sink().stats();
        health_.flight_dump_failures = stats.dumps_failed;
        health_.flight_evictions = stats.evicted_by_capacity;
        health_.flight_stored_bytes = stats.stored_bytes;
        health_.flight_stored_snapshots = stats.stored_snapshots;
    }
    std::deque<DataTamer2::MCAPRingDump> completed;
    {
        std::lock_guard<std::mutex> lock(dump_mutex_);
        completed.swap(dumps_);
        health_.flight_report_drops = callback_drops_;
    }
    for (const auto& dump : completed) {
        YAML::Node report;
        if (!dump_requests_.empty()) {
            report["request_ns"] = dump_requests_.front().first;
            report["trigger_code"] = dump_requests_.front().second;
            dump_requests_.pop_front();
        }
        report["configured_pre_seconds"] = pre_sec_;
        report["configured_post_seconds"] = post_sec_;
        report["post_interval_complete"] =
            dump.last_message + Nanos(1 / rate_hz_) >= dump.trigger_time + Nanos(post_sec_);
        report["path"] = dump.path;
        report["ok"] = dump.ok;
        report["error"] = dump.error;
        report["trigger_ns"] = dump.trigger_time.count();
        report["requested_start_ns"] = dump.start.count();
        report["requested_end_ns"] = dump.end.count();
        report["first_message_ns"] = dump.first_message.count();
        report["last_message_ns"] = dump.last_message.count();
        report["messages"] = dump.messages;
        report["truncated"] = dump.truncated;
        WriteJson(dump.path + ".json", report);
        Event(dump.ok ? "FLIGHT_DUMP_OK" : "FLIGHT_DUMP_FAILED", dump.path + " " + dump.error);
        if (!dump.ok)
            Fault("flight dump failed: " + dump.error);
    }
    health_.disk_available_bytes = storage_->Available();
    if (state_channel_ && health_.state == "RECORDING") {
        try {
            std::set<fs::path> active;
            if (continuous_)
                active.insert(active_file_);
            if (flight_)
                active.insert(storage_->Directory() / "faults");
            storage_->Enforce(active);
            if (continuous_ && (now - segment_start_ns_ >= std::uint64_t(rotation_sec_ * 1e9) ||
                                   fs::file_size(active_file_) >= segment_bytes_)) {
                const auto next = storage_->Directory() /
                                  ("recording_" + std::to_string(++segment_number_) + ".mcap");
                if (fs::exists(next))
                    throw std::runtime_error("rotation path already exists");
                continuous_->sink().restartRecording(
                    next.string(), config_["continuous"]["compression"].as<bool>());
                active_file_ = next;
                segment_start_ns_ = now;
                Event("FILE_ROTATED", next.filename().string());
            }
        } catch (const std::exception& e) {
            storage_writable_ = false;
            Fault(e.what());
        }
    }
    if (health_channel_ && collecting_ &&
        !DataTamer2::accepted(health_channel_->tryTakeSnapshot(std::chrono::nanoseconds(now))))
        ++auxiliary_failures_;
}

YAML::Node LoggerService::Status() const {
    YAML::Node out;
    out["ok"] = true;
    out["state"] = health_.state;
    out["enabled"] = config_["enabled"].as<bool>();
    out["dump_storage_available"] = storage_writable_;
    out["last_error"] = health_.last_error;
    out["shm_name"] = shm_name_;
    out["socket"] = socket_path_.string();
    out["session_directory"] = storage_->Directory().string();
    out["session_number"] = session_number_;
    out["snapshot_attempts"] = health_.snapshot_attempts;
    out["snapshot_successes"] = health_.snapshot_successes;
    out["snapshot_failures"] = health_.snapshot_failures;
    out["shm_lock_misses"] = health_.shm_lock_misses;
    out["mutex_recoveries"] = health_.mutex_recoveries;
    out["abi_failures"] = health_.abi_failures;
    out["source_regressions"] = health_.source_regressions;
    out["invalid_timestamps"] = health_.invalid_timestamps;
    out["last_successful_snapshot_ns"] = health_.last_successful_snapshot_ns;
    out["schedule_skips"] = health_.schedule_skips;
    out["mutex_wait_average_us"] = health_.mutex_wait_average_us;
    out["mutex_wait_max_us"] = health_.mutex_wait_max_us;
    out["mutex_hold_average_us"] = health_.mutex_hold_average_us;
    out["mutex_hold_max_us"] = health_.mutex_hold_max_us;
    out["estimated_input_rate_hz"] = health_.input_rate_hz;
    out["accepted_output_rate_hz"] = health_.output_rate_hz;
    out["drop_rate"] = health_.drop_rate;
    out["datatamer_attempts"] = health_.datatamer_attempts;
    out["datatamer_accepted"] = health_.datatamer_accepted;
    out["pool_exhausted"] = health_.pool_exhausted;
    out["oversize"] = health_.oversize;
    out["serialization_failures"] = health_.serialization_failures;
    out["blocked_snapshots"] = health_.blocked_snapshots;
    out["sink_delivered"] = health_.sink_delivered;
    out["sink_errors"] = health_.sink_errors;
    out["queue_high_water"] = health_.queue_high_water;
    out["disk_available_bytes"] = health_.disk_available_bytes;
    out["flight_dump_failures"] = health_.flight_dump_failures;
    out["triggers_accepted"] = health_.triggers_accepted;
    out["triggers_rejected"] = health_.triggers_rejected;
    out["flight_evictions"] = health_.flight_evictions;
    out["flight_report_drops"] = health_.flight_report_drops;
    out["flight_stored_bytes"] = health_.flight_stored_bytes;
    out["flight_stored_snapshots"] = health_.flight_stored_snapshots;
    for (const auto value : health_.source_sequence_gaps)
        out["source_sequence_gaps"].push_back(value);
    for (int i = 0; i < 4; ++i) {
        out["source_valid"].push_back(health_.source_valid[i]);
        if (health_.source_valid[i])
            out["source_age_ms"].push_back(health_.source_age_ms[i]);
        else
            out["source_age_ms"].push_back(YAML::Node());
    }
    out["mutex_scope"] = "logger trylock/copy/unlock only; NOT producer wait/hold";
    out["unavailable_metrics"] =
        "true controller deadline misses, producer mutex timing, independent heartbeat";
    out["sequence_gap_meaning"] = "publications missed by this observer; NOT CAN packet loss";
    return out;
}

void LoggerService::Stop(const std::string& result) {
    if (!state_channel_ && !continuous_ && !flight_ && !sampler_.joinable())
        return;
    collecting_ = false;
    if (sampler_.joinable())
        sampler_.join();
    health_.state = "STOPPING";
    Event("RECORDING_STOPPING", result);
    if (continuous_)
        continuous_->stop();
    if (flight_)
        flight_->stop();  // Pending post-trigger interval is cut and its real bounds reported.
    UpdateHealth();
    auto quality = Status();
    quality["result"] = health_.last_error.empty() ? result : "degraded";
    quality["duration_seconds"] = (shm_utils::MonotonicNs() - start_ns_) / 1e9;
    quality["recording_quality_scope"] =
        "accepted vs rejected snapshots; use qhrr-log-inspect for on-disk counts";
    try {
        storage_->Finish(quality["result"].as<std::string>(), quality);
    } catch (const std::exception& e) {
        health_.last_error = e.what();
        std::cerr << "[shm_logger] metadata finalization ERROR: " << e.what() << '\n';
    }
    session_channel_.reset();
    event_channel_.reset();
    health_channel_.reset();
    state_channel_.reset();
    continuous_.reset();
    flight_.reset();
    simulation_id_.reset();
    health_.state = source_fault_ ? "FAULT" : "READY";
}

YAML::Node LoggerService::Command(const std::string& request) {
    if (request == "status" || request == "health")
        return Status();
    if (request == "start") {
        Start();
        return Status();
    }
    if (request == "stop") {
        Stop("normal");
        auto_start_ = false;
        return Status();
    }
    if (request == "trigger") {
        if (!collecting_)
            throw std::runtime_error("no active recording");
        const bool accepted = Trigger("MANUAL_TRIGGER");
        YAML::Node out;
        out["ok"] = accepted;
        out["message"] = accepted ? "dump accepted" : "dump disabled or already pending";
        return out;
    }
    if (request.rfind("mark ", 0) == 0) {
        const auto text = request.substr(5);
        if (!collecting_ || text.empty() || text.size() > 512)
            throw std::runtime_error("mark requires an active recording and 1..512 bytes");
        Event("USER_MARK", text);
        YAML::Node out;
        out["ok"] = true;
        return out;
    }
    if (request == "sessions") {
        YAML::Node out;
        out["ok"] = true;
        out["sessions"] = YAML::Node(YAML::NodeType::Sequence);
        std::vector<std::string> names;
        for (const auto& entry : fs::directory_iterator(storage_->Root()))
            if (entry.is_directory() && fs::is_regular_file(entry.path() / "session.json"))
                names.push_back(entry.path().filename().string());
        std::sort(names.rbegin(), names.rend());
        out["truncated"] = names.size() > 100;
        for (std::size_t i = 0; i < std::min<std::size_t>(100, names.size()); ++i)
            out["sessions"].push_back(names[i]);
        return out;
    }
    throw std::runtime_error("unknown command: use status/start/stop/trigger/mark/sessions/health");
}

void LoggerService::PollCommands() {
    for (int count = 0; count < 8; ++count) {
        const int client = ::accept4(socket_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
        if (client < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                return;
            shm_utils::Check(errno, "logger control accept");
        }
        YAML::Node reply;
        try {
            ucred peer{};
            socklen_t length = sizeof(peer);
            if (::getsockopt(client, SOL_SOCKET, SO_PEERCRED, &peer, &length) != 0 ||
                peer.uid != ::getuid())
                throw std::runtime_error("logger control peer is not the owning user");
            pollfd readable{client, POLLIN, 0};
            if (::poll(&readable, 1, 20) <= 0)
                throw std::runtime_error("logger command packet timeout");
            std::array<char, 4097> packet{};
            const auto size = ::recv(client, packet.data(), packet.size(), MSG_TRUNC);
            if (size <= 0 || size >= static_cast<ssize_t>(packet.size()))
                throw std::runtime_error("invalid/oversized logger command packet");
            reply = Command(std::string(packet.data(), size));
        } catch (const std::exception& e) {
            reply["ok"] = false;
            reply["error"] = e.what();
        }
        const auto response = Json(reply);
        if (::send(client, response.data(), response.size(), MSG_NOSIGNAL) !=
            static_cast<ssize_t>(response.size()))
            std::cerr << "[shm_logger] control reply ERROR: " << std::strerror(errno) << '\n';
        ::close(client);
    }
}

int LoggerService::Run(volatile std::sig_atomic_t& stop) {
    OpenSocket();
    Daemon::NotifyReady();
    std::cout << "[shm_logger] waiting for SHM (timeout " << attach_timeout_sec_ << " s)\n";
    const auto deadline =
        shm_utils::MonotonicNs() + static_cast<std::uint64_t>(attach_timeout_sec_ * 1e9);
    auto next_health = shm_utils::MonotonicNs(), next_display = next_health;
    bool attempted_start = false;
    while (!stop) {
        if (!reader_ && health_.state == "INITIALIZING") {
            const int fd = ::shm_open(shm_name_.c_str(), O_RDONLY | O_CLOEXEC, kShmPermissions);
            if (fd >= 0) {
                ::close(fd);
                try {
                    Attach();
                } catch (const std::exception& e) {
                    ++health_.abi_failures;
                    Fault(e.what());
                }
            } else if (errno != ENOENT || shm_utils::MonotonicNs() >= deadline) {
                Fault(std::string("SHM attach failed: ") + std::strerror(errno));
            }
        }
        if (reader_ && auto_start_ && !attempted_start) {
            attempted_start = true;
            try {
                Start();
            } catch (const std::exception& e) {
                Fault(e.what());
            }
        }
        PollCommands();
        const auto now = shm_utils::MonotonicNs();
        if (now >= next_health) {
            try {
                UpdateHealth();
            } catch (const std::exception& e) {
                Fault(e.what());
            }
            next_health = now + static_cast<std::uint64_t>(1e9 / health_hz_);
        }
        if (now >= next_display) {
            const bool framed = std::getenv("QHRR_TERMINAL_STATISTICS") != nullptr;
            if (framed)
                std::cout << Daemon::kStatisticsBegin;
            std::cout << "LOGGER | " << health_.state
                      << " | accepted=" << health_.snapshot_successes
                      << " lock_miss=" << health_.shm_lock_misses
                      << " output=" << health_.output_rate_hz << " Hz\n"
                      << "trylock mean/max " << health_.mutex_wait_average_us << '/'
                      << health_.mutex_wait_max_us << " us | copy+unlock "
                      << health_.mutex_hold_average_us << '/' << health_.mutex_hold_max_us
                      << " us (logger only)\n"
                      << "pool drops=" << health_.pool_exhausted
                      << " sink errors=" << health_.sink_errors
                      << " flight failed=" << health_.flight_dump_failures
                      << " disk=" << health_.disk_available_bytes << " bytes\n";
            if (!health_.last_error.empty())
                std::cout << "ERROR: " << health_.last_error << '\n';
            if (framed)
                std::cout << Daemon::kStatisticsEnd;
            std::cout.flush();
            next_display = now + static_cast<std::uint64_t>(display_sec_ * 1e9);
        }
        bool finished = false;
        {
            std::lock_guard<std::mutex> lock(latest_mutex_);
            finished =
                latest_.stopping || (runtime_mode_ == "simulation" && latest_.simulation_finished);
        }
        if (finished)
            break;
        pollfd wait{socket_fd_, POLLIN, 0};
        const int result = ::poll(&wait, 1, 10);
        if (result < 0 && errno != EINTR)
            shm_utils::Check(errno, "logger management poll");
    }
    Stop(stop ? "interrupted" : "normal");
    return 0;
}
}  // namespace qhrr05::logging
